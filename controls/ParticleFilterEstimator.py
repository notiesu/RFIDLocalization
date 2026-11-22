"""Constant-velocity particle filter described in ControlModel.jpg.

The C++ estimator was empty. This implementation uses independent Gaussian
acceleration, joint RSSI/bearing likelihoods, and systematic resampling.
"""

from copy import deepcopy
from math import atan2, cos, exp, hypot, log, log10, sin
from random import Random

from .Estimator import Estimator
from .EstimatorDiagnostics import EstimatorDiagnostics
from .EstimatorLogger import EstimatorLogger
from .RFIDSensor import RFIDConfig
from .Utils import Coord, Particle, RFIDMeasurement, ReaderState, TagState, positive, wrapAngle


class ParticleFilterEstimator(Estimator[TagState, RFIDMeasurement, ReaderState]):
    def __init__(
        self,
        config: RFIDConfig,
        particleCount: int = 500,
        accelerationStdDev: float = 0.5,
        resamplingThreshold: float = 0.5,
        seed: int = 1,
        initialState: TagState | None = None,
        logger: EstimatorLogger | None = None,
    ):
        if not isinstance(particleCount, int) or particleCount < 2:
            raise ValueError("particleCount must be an integer >= 2")
        positive(accelerationStdDev, "accelerationStdDev", allowZero=True)
        if not 0 < resamplingThreshold <= 1:
            raise ValueError("resamplingThreshold must lie in (0, 1]")
        self.config = deepcopy(config)
        self.particleCount = particleCount
        self.accelerationStdDev = accelerationStdDev
        self.resamplingThreshold = resamplingThreshold  # Fraction of particle count
        self.rng = Random(seed)
        self.particleField: list[Particle] = []
        self.logger = logger if logger is not None else EstimatorLogger()
        self._nextTime = 0.0
        if initialState is not None:
            self.initializeParticles(initialState)

    @property
    def diagnostics(self) -> EstimatorDiagnostics:
        return self.logger.latestDiagnostics

    @property
    def initialized(self) -> bool:
        return bool(self.particleField)

    def initializeParticles(
        self, initialState: TagState, positionStdDev: float = 1.0, velocityStdDev: float = 1.0
    ) -> None:
        positive(positionStdDev, "positionStdDev", allowZero=True)
        positive(velocityStdDev, "velocityStdDev", allowZero=True)
        self.particleField = [
            Particle(
                TagState(
                    Coord(self.rng.gauss(initialState.position.x, positionStdDev),
                          self.rng.gauss(initialState.position.y, positionStdDev)),
                    self.rng.gauss(initialState.xdot, velocityStdDev),
                    self.rng.gauss(initialState.ydot, velocityStdDev),
                ),
                1.0 / self.particleCount, #equal weighting as initial
            )
            for _ in range(self.particleCount)
        ]

    def _initializeFromMeasurement(self, measurement: RFIDMeasurement, reader: ReaderState) -> None:
        # Bootstrap from the observation, never from simulation ground truth.
        c = self.config
        self.particleField = []
        for _ in range(self.particleCount):
            #for every particle
            rssi = self.rng.gauss(measurement.Pr, measurement.uncertainty)
            exponent = (c.referenceRSSI - rssi) / (10.0 * c.pathLossExponent)
            maxExponent = log10(c.maxRange / c.referenceDistance)
            distance = c.referenceDistance * 10.0 ** min(exponent, maxExponent)
            if distance <= c.referenceDistance:
                # The sensor clamps RSSI inside d0, so it cannot resolve range there.
                distance = min(c.referenceDistance, c.maxRange) * self.rng.random() ** 0.5
            bearing = reader.heading + self.rng.gauss(measurement.bearing, c.bearingStdDev)
            state = TagState(
                Coord(reader.position.x + distance * cos(bearing),
                      reader.position.y + distance * sin(bearing)),
                self.rng.gauss(0.0, 1.0), self.rng.gauss(0.0, 1.0),
            )
            self.particleField.append(Particle(state, 1.0 / self.particleCount))

    def getNextParticleState(self, state: TagState, dt: float) -> TagState:
        positive(dt, "dt")
        ax = self.rng.gauss(0.0, self.accelerationStdDev)
        ay = self.rng.gauss(0.0, self.accelerationStdDev)
        return TagState(
            Coord(state.position.x + state.xdot * dt + 0.5 * ax * dt * dt,
                  state.position.y + state.ydot * dt + 0.5 * ay * dt * dt),
            state.xdot + ax * dt,
            state.ydot + ay * dt,
        )

    def predictParticles(self, dt: float) -> None:
        positive(dt, "dt")
        for particle in self.particleField:
            particle.state = self.getNextParticleState(particle.state, dt)

    def updateWeights(self, measurement: RFIDMeasurement, reader: ReaderState) -> None:
        if not measurement.isValid or not self.initialized:
            return
        c = self.config
        # Zero-noise simulations still need a finite numerical likelihood width.
        rssiSigma = max(measurement.uncertainty, 1e-6)
        bearingSigma = max(c.bearingStdDev, 1e-6)
        logWeights = []
        for particle in self.particleField:
            dx = particle.state.position.x - reader.position.x
            dy = particle.state.position.y - reader.position.y
            distance = max(hypot(dx, dy), c.referenceDistance)
            expectedRSSI = c.referenceRSSI - 10.0 * c.pathLossExponent * log10(distance / c.referenceDistance)
            expectedBearing = wrapAngle(atan2(dy, dx) - reader.heading)
            rssiError = (measurement.Pr - expectedRSSI) / rssiSigma
            bearingError = wrapAngle(measurement.bearing - expectedBearing) / bearingSigma
            prior = log(particle.weight) if particle.weight > 0 else -float("inf")
            logWeights.append(prior - 0.5 * (rssiError ** 2 + bearingError ** 2))
        peak = max(logWeights)
        weights = [exp(value - peak) for value in logWeights]
        total = sum(weights)
        for particle, weight in zip(self.particleField, weights):
            particle.weight = weight / total

    def calculateStateEstimate(self) -> TagState:
        if not self.initialized:
            raise RuntimeError("Estimator needs an initial prior or a valid RFID measurement")
        return TagState(
            Coord(sum(p.weight * p.state.position.x for p in self.particleField),
                  sum(p.weight * p.state.position.y for p in self.particleField)),
            sum(p.weight * p.state.xdot for p in self.particleField),
            sum(p.weight * p.state.ydot for p in self.particleField),
        )

    def calculateEffectiveSampleSize(self) -> float:
        return 1.0 / sum(p.weight ** 2 for p in self.particleField) if self.initialized else 0.0

    def resampleParticles(self) -> None:
        if not self.initialized:
            return
        count = len(self.particleField)
        offset = self.rng.random() / count
        cumulative = self.particleField[0].weight
        index = 0
        particles = []
        for i in range(count):
            target = offset + i / count
            while target > cumulative and index < count - 1:
                index += 1
                cumulative += self.particleField[index].weight
            particles.append(Particle(deepcopy(self.particleField[index].state), 1.0 / count))
        self.particleField = particles

    def update(
        self, measurement: RFIDMeasurement | None, reader: ReaderState, dt: float,
        *, time: float | None = None, predict: bool = True,
    ) -> TagState | None:
        """Predict once per tick, then consume each fresh measurement once.

        None means no sensor sample was due. Invalid samples represent dropouts.
        Until acquisition, return None so the simulation can hold the reader still.
        History stores the weighted cloud before resampling. Explicit time aligns
        snapshots with the simulation; direct calls start at t=0 and advance by dt.
        Set predict=False for a prior that already describes the current time.
        """
        positive(dt, "dt")
        time = self._nextTime if time is None else time
        positive(time, "time", allowZero=True)
        if not self.initialized:
            if measurement is not None and measurement.isValid:
                self._initializeFromMeasurement(measurement, reader)
        else:
            if predict:
                self.predictParticles(dt)
            if measurement is not None:
                self.updateWeights(measurement, reader)
        estimate = self.calculateStateEstimate() if self.initialized else None
        ess = self.calculateEffectiveSampleSize()
        resampled = bool(self.initialized and predict and ess < self.resamplingThreshold * self.particleCount)
        self.logger.log(time, self.particleField, estimate, reader, measurement, ess, resampled)
        if resampled:
            self.resampleParticles()
        self._nextTime = time + dt
        return estimate
