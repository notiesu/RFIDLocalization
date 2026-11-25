"""Nested parameter/state particle inference from RFID and observed inputs.

SMC²-style parameter reweighting and particle marginal Metropolis moves retain
static parameter hypotheses without adding artificial parameter drift. Each
parameter hypothesis carries a bootstrap motion filter. Replays use only past
observations. Sensor calibration is supplied, hidden motion parameters are not.
"""

from copy import deepcopy
from dataclasses import dataclass
from math import isfinite

import numpy as np

from .Estimator import Estimator
from .EstimatorDiagnostics import EstimatorDiagnostics
from .ParameterParticleFilterEstimator import ParameterParticleFilterEstimator, ParameterPrior
from .RFIDSensor import RFIDConfig
from .Utils import Coord, RFIDMeasurement, ReaderState, TagState, positive


@dataclass(frozen=True)
class JointFilterContext:
    reader: ReaderState
    # Inputs on the interval ENDING at this reading, not the outgoing interval.
    inputs: tuple[float, float] | None = None
    instruments: tuple[float, float] | None = None

    def inputArrays(self):
        u, z = np.asarray(self.inputs, dtype=float), np.asarray(self.instruments, dtype=float)
        if u.shape != (2,) or z.shape != (2,) or not np.all(np.isfinite([u, z])):
            raise ValueError("Joint prediction requires finite two-axis inputs and instruments")
        return u, z


@dataclass(frozen=True)
class StatePrediction:
    mean: TagState
    covariance: np.ndarray  # x,y,vx,vy; returned independently of filter storage.


@dataclass(frozen=True)
class JointEstimate:
    time: float
    state: TagState
    beta: float
    betaStddev: float
    lower95: float
    upper95: float
    parameterESS: float
    resampled: bool
    moveAcceptanceRate: float | None


class JointParticleFilterEstimator(Estimator[TagState, RFIDMeasurement, JointFilterContext]):
    """Learn motion and beta without acceleration/velocity truth observations.

    independent: acceleration = beta*u + Gaussian residual.
    shared: u=alpha*z+r; acceleration=beta*u+lambda*r+Gaussian residual.
    All coefficients and residual scales have generic priors and are learned.
    As in ParameterParticleFilterEstimator, the Gaussian likelihood is a working
    approximation to the non-Gaussian generator, and beta is constant over a run.

    Initialization uses the first valid RFID reading as an empirical state prior;
    it is not counted again as a likelihood update. Pre-acquisition intervals
    are discarded. Invalid reads are treated as uninformative, as in the tracker.
    This standalone estimator needs input contexts; Simulation's old ReaderState
    interface is deliberately not changed.
    """

    def __init__(self, config: RFIDConfig, *, mode: str = "independent",
                 parameterCount: int = 256, stateParticleCount: int = 128,
                 prior: ParameterPrior | None = None, resamplingThreshold: float = 0.5,
                 moveSteps: int = 2, initialVelocityStddev: float = 1.0, seed: int = 1):
        # Reuse the parameter prior/space implementation; no observed-acceleration
        # filter is run, and none of its sufficient statistics are used here.
        space = ParameterParticleFilterEstimator(mode=mode, particleCount=parameterCount,
                                                 prior=prior, resamplingThreshold=resamplingThreshold,
                                                 moveSteps=moveSteps, seed=seed)
        if not isinstance(stateParticleCount, int) or stateParticleCount < 2:
            raise ValueError("stateParticleCount must be an integer >= 2")
        positive(initialVelocityStddev, "initialVelocityStddev")
        self.config, self.mode = deepcopy(config), mode
        self.parameterCount, self.stateParticleCount = parameterCount, stateParticleCount
        self.parameterNames, self.prior = space.parameterNames, space.prior
        self._parameters, self._bounds = space.parameterParticles, space._bounds.copy()
        self._parameterWeights = space.weights
        self.resamplingThreshold, self.moveSteps = resamplingThreshold, moveSteps
        self.initialVelocityStddev = initialVelocityStddev
        self.rng = np.random.default_rng(seed + 1)
        self._states = self._stateWeights = self._initialPool = None
        self._likelihood = np.zeros(parameterCount)
        self._records = []
        self._nextTime = 0.0
        self.lastPrediction: StatePrediction | None = None
        self.history: list[JointEstimate] = []
        self._diagnostics = EstimatorDiagnostics()

    @property
    def initialized(self):
        return self._states is not None

    @property
    def diagnostics(self):
        return self._diagnostics

    @property
    def parameterParticles(self):
        return self._parameters.copy()

    @property
    def parameterWeights(self):
        return self._parameterWeights.copy()

    @property
    def stateParticles(self):
        return None if not self.initialized else self._states.copy()

    @property
    def stateWeights(self):
        return None if not self.initialized else self._stateWeights.copy()

    @staticmethod
    def _normalize(logWeights, axis=-1):
        normalizer = np.logaddexp.reduce(logWeights, axis=axis, keepdims=True)
        if not np.all(np.isfinite(normalizer)):
            raise ValueError("Non-finite RFID likelihood; check sensor calibration and priors")
        return np.exp(logWeights - normalizer), np.squeeze(normalizer, axis=axis)

    @staticmethod
    def _state(mean):
        return TagState(Coord(float(mean[0]), float(mean[1])), float(mean[2]), float(mean[3]))

    @staticmethod
    def _moments(states, weights):
        flat, weights = states.reshape(-1, 4), weights.ravel()
        mean = weights @ flat
        centered = flat - mean
        return StatePrediction(JointParticleFilterEstimator._state(mean),
                               (centered * weights[:, None]).T @ centered)

    def calculateStateEstimate(self):
        if not self.initialized:
            raise RuntimeError("Joint filter needs a valid RFID reading")
        weights = self._parameterWeights[:, None] * self._stateWeights
        return self._moments(self._states, weights).mean

    def _initialStates(self, count):
        indices = self.rng.integers(len(self._initialPool), size=(count, self.stateParticleCount))
        return self._initialPool[indices].copy(), np.full(indices.shape, 1 / self.stateParticleCount)

    def _initialize(self, measurement, reader):
        c = self.config
        count = 4096
        noisyRSSI = self.rng.normal(measurement.Pr, max(measurement.uncertainty, 1e-6), count)
        exponent = (c.referenceRSSI - noisyRSSI) / (10 * c.pathLossExponent)
        distance = c.referenceDistance * 10**np.minimum(exponent, np.log10(c.maxRange / c.referenceDistance))
        near = distance <= c.referenceDistance
        distance[near] = min(c.referenceDistance, c.maxRange) * np.sqrt(self.rng.random(np.count_nonzero(near)))
        bearing = reader.heading + self.rng.normal(measurement.bearing, max(c.bearingStdDev, 1e-6), count)
        self._initialPool = np.column_stack((reader.position.x + distance*np.cos(bearing),
                                             reader.position.y + distance*np.sin(bearing),
                                             self.rng.normal(0, self.initialVelocityStddev, (count, 2))))
        self._states, self._stateWeights = self._initialStates(self.parameterCount)

    def _inputLikelihood(self, parameters, u, z):
        if self.mode == "independent":
            return np.zeros(len(parameters))
        residual = u - parameters[:, 1, None] * z
        logScale = parameters[:, 3]
        return -2 * (logScale + 0.5*np.log(2*np.pi)) - 0.5*np.sum(residual**2, axis=1)*np.exp(-2*logScale)

    def _acceleration(self, parameters, u, z):
        acceleration = parameters[:, 0, None] * u
        if self.mode == "shared":
            acceleration += parameters[:, 2, None] * (u - parameters[:, 1, None]*z)
        return acceleration, np.exp(parameters[:, -1])

    def predictNextState(self, context: JointFilterContext, dt: float) -> StatePrediction:
        """Forecast using an observed outgoing input, without mutating the filter.

        Does not consume a future RFID reading. Shared-mode weights condition on
        the newly supplied input as well as instrument; context.reader is unused.
        """
        positive(dt, "dt")
        if not self.initialized:
            raise RuntimeError("Joint filter needs a valid RFID reading")
        u, z = context.inputArrays()
        with np.errstate(divide="ignore"):
            weights, _ = self._normalize(np.log(self._parameterWeights)
                                         + self._inputLikelihood(self._parameters, u, z))
        acceleration, sigma = self._acceleration(self._parameters, u, z)
        states = self._states.copy()
        states[:, :, :2] += states[:, :, 2:] * dt
        states[:, :, 2:] += acceleration[:, None, :] * dt
        prediction = self._moments(states, weights[:, None] * self._stateWeights)
        # New process noise affects velocity only under the supplied Euler model.
        prediction.covariance[2:, 2:] += np.eye(2) * float(weights @ sigma**2) * dt**2
        return prediction

    def _measurementLikelihood(self, states, measurement, reader):
        c = self.config
        dx, dy = states[:, :, 0] - reader.position.x, states[:, :, 1] - reader.position.y
        distance = np.maximum(np.hypot(dx, dy), c.referenceDistance)
        expectedRSSI = c.referenceRSSI - 10*c.pathLossExponent*np.log10(distance/c.referenceDistance)
        bearing = np.arctan2(dy, dx) - reader.heading
        angleError = measurement.bearing - bearing
        angleError = np.arctan2(np.sin(angleError), np.cos(angleError))
        rssiSigma, bearingSigma = max(measurement.uncertainty, 1e-6), max(c.bearingStdDev, 1e-6)
        return (-0.5*((measurement.Pr-expectedRSSI)/rssiSigma)**2
                -0.5*(angleError/bearingSigma)**2 - np.log(2*np.pi*rssiSigma*bearingSigma))

    def _advance(self, parameters, states, weights, record):
        measurement, context, dt, predict = record
        likelihood = np.zeros(len(parameters))
        if predict:
            u, z = context.inputArrays()
            likelihood += self._inputLikelihood(parameters, u, z)
            acceleration, sigma = self._acceleration(parameters, u, z)
            states[:, :, :2] += states[:, :, 2:] * dt
            states[:, :, 2:] += dt*(acceleration[:, None, :] + sigma[:, None, None]
                                    * self.rng.normal(size=states[:, :, 2:].shape))
        if measurement is not None and measurement.isValid:
            with np.errstate(divide="ignore"):
                weights, increment = self._normalize(np.log(weights)
                    + self._measurementLikelihood(states, measurement, context.reader))
            likelihood += increment
        # Independent systematic resampling for each inner filter when needed.
        rows = np.flatnonzero(1 / np.sum(weights**2, axis=1) < 0.5*self.stateParticleCount)
        if len(rows):
            cumulative = np.cumsum(weights[rows], axis=1)
            cumulative[:, -1] = 1
            offsets = 2*np.arange(len(rows))[:, None]
            targets = (self.rng.random((len(rows), 1)) + np.arange(self.stateParticleCount)) / self.stateParticleCount
            indices = np.searchsorted((cumulative+offsets).ravel(), (targets+offsets).ravel()).reshape(targets.shape) % self.stateParticleCount
            states[rows] = states[rows[:, None], indices]
            weights[rows] = 1 / self.stateParticleCount
        return states, weights, likelihood

    def _replay(self, parameters):
        states, weights = self._initialStates(len(parameters))
        likelihood = np.zeros(len(parameters))
        for record in self._records:
            states, weights, increment = self._advance(parameters, states, weights, record)
            likelihood += increment
        return states, weights, likelihood

    def _resampleAndMove(self, proposalCholesky):
        cumulative = np.cumsum(self._parameterWeights)
        cumulative[-1] = 1
        targets = (self.rng.random() + np.arange(self.parameterCount)) / self.parameterCount
        indices = np.searchsorted(cumulative, targets)
        self._parameters, self._states = self._parameters[indices].copy(), self._states[indices].copy()
        self._stateWeights, self._likelihood = self._stateWeights[indices].copy(), self._likelihood[indices].copy()
        self._parameterWeights.fill(1 / self.parameterCount)
        accepted = 0
        for _ in range(self.moveSteps):
            proposal = self._parameters + self.rng.normal(size=self._parameters.shape) @ proposalCholesky.T
            rows = np.flatnonzero(np.all((proposal >= self._bounds[:, 0]) & (proposal <= self._bounds[:, 1]), axis=1))
            if not len(rows):
                continue
            states, weights, likelihood = self._replay(proposal[rows])
            accept = np.log(self.rng.random(len(rows))) < likelihood - self._likelihood[rows]
            selected = rows[accept]
            self._parameters[selected] = proposal[selected]
            self._states[selected], self._stateWeights[selected] = states[accept], weights[accept]
            self._likelihood[selected] = likelihood[accept]
            accepted += len(selected)
        return accepted / (self.moveSteps*self.parameterCount)

    def update(self, measurement: RFIDMeasurement | None, context: JointFilterContext,
               dt: float, *, time: float | None = None, predict: bool = True) -> TagState | None:
        positive(dt, "dt")
        time = self._nextTime if time is None else time
        positive(time, "time", allowZero=True)
        if not all(isfinite(v) for v in (context.reader.position.x, context.reader.position.y, context.reader.heading)):
            raise ValueError("Reader pose must be finite")
        if measurement is not None and measurement.isValid:
            if not all(isfinite(v) for v in (measurement.Pr, measurement.bearing)):
                raise ValueError("Valid RFID readings must be finite")
            positive(measurement.uncertainty, "RFID uncertainty", allowZero=True)
        # Copy records so caller mutations cannot alter later likelihood replays.
        record = deepcopy((measurement, context, dt, predict))
        if self.initialized and predict:
            context.inputArrays()
        self._nextTime = time + dt
        self.lastPrediction = None
        ess, resampled, acceptance = float(self.parameterCount), False, None
        if not self.initialized:
            if measurement is None or not measurement.isValid:
                return None
            self._initialize(measurement, context.reader)
        else:
            if predict:
                self.lastPrediction = self.predictNextState(context, dt)
            self._states, self._stateWeights, increment = self._advance(
                self._parameters, self._states, self._stateWeights, record)
            self._likelihood += increment
            self._records.append(record)
            with np.errstate(divide="ignore"):
                self._parameterWeights, _ = self._normalize(np.log(self._parameterWeights) + increment)
            ess = float(1 / np.sum(self._parameterWeights**2))
            resampled = ess < self.resamplingThreshold*self.parameterCount
            if resampled:
                centered = self._parameters - self._parameterWeights @ self._parameters
                covariance = (centered*self._parameterWeights[:, None]).T @ centered
                covariance += np.diag(np.diff(self._bounds, axis=1).ravel()**2*1e-8)
                proposal = np.linalg.cholesky(covariance)*(2.38/np.sqrt(len(self.parameterNames)))
                acceptance = self._resampleAndMove(proposal)
        jointWeights = self._parameterWeights[:, None]*self._stateWeights
        estimate = self._moments(self._states, jointWeights)
        beta = float(self._parameterWeights @ self._parameters[:, 0])
        betaStddev = float(np.sqrt(self._parameterWeights @ (self._parameters[:, 0]-beta)**2))
        order = np.argsort(self._parameters[:, 0])
        indices = np.searchsorted(np.cumsum(self._parameterWeights[order]), [0.025, 0.975])
        lower, upper = self._parameters[order[np.minimum(indices, self.parameterCount-1)], 0]
        self.history.append(JointEstimate(time, deepcopy(estimate.mean), beta, betaStddev,
                                         float(lower), float(upper), ess, resampled, acceptance))
        self._diagnostics = EstimatorDiagnostics(float(estimate.covariance[0, 0]), float(estimate.covariance[0, 1]),
            float(estimate.covariance[1, 1]), float(1/np.sum(jointWeights**2)), resampled)
        return estimate.mean
