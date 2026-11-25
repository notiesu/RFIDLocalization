"""Sequential Monte Carlo inference for a static acceleration coefficient beta.

This estimates parameters from observed acceleration, not hidden RFID states.
Systematic resampling plus posterior-invariant Metropolis moves avoids treating
a static parameter as a random walk. Noise and input-model parameters are inferred
jointly; no generator parameters or observation-noise truth are accepted.
"""

from dataclasses import dataclass
from math import isfinite

import numpy as np

from .Estimator import Estimator
from .Utils import positive


@dataclass(frozen=True)
class ParameterObservation:
    acceleration: tuple[float, ...]
    inputs: tuple[float, ...]
    instruments: tuple[float, ...]

    def arrays(self):
        values = [np.asarray(v, dtype=float) for v in
                  (self.acceleration, self.inputs, self.instruments)]
        if (any(v.ndim != 1 or not len(v) or not np.all(np.isfinite(v)) for v in values)
                or len({v.shape for v in values}) != 1):
            raise ValueError("Acceleration, inputs, and instruments need equal nonempty finite vectors")
        return values


@dataclass(frozen=True)
class ParameterPrior:
    """Fixed inference choices, never constructed from generator metadata.

    Standard-deviation bounds specify a log-uniform prior. confoundingSlope is
    a control-function coefficient, not the generator's gamma or delta.
    """

    beta: tuple[float, float] = (-3.0, 3.0)
    inputResponse: tuple[float, float] = (-3.0, 3.0)
    confoundingSlope: tuple[float, float] = (-3.0, 3.0)
    noiseStddev: tuple[float, float] = (0.01, 10.0)

    def __post_init__(self):
        for name in ("beta", "inputResponse", "confoundingSlope", "noiseStddev"):
            bounds = getattr(self, name)
            if len(bounds) != 2 or not all(isfinite(v) for v in bounds) or bounds[0] >= bounds[1]:
                raise ValueError(f"{name} prior bounds must be finite and increasing")
        if self.noiseStddev[0] <= 0:
            raise ValueError("noiseStddev prior bounds must be positive")


@dataclass(frozen=True)
class ParameterEstimate:
    time: float
    beta: float
    stddev: float
    lower95: float
    upper95: float
    effectiveSampleSize: float
    resampled: bool
    moveAcceptanceRate: float | None
    observationCount: int


class ParameterParticleFilterEstimator(Estimator[float, ParameterObservation, None]):
    """Static-parameter SMC with all noise parameters learned from observations.

    independent: a = beta*u + e, with unknown Gaussian residual scale. This
      deliberately ignores input/disturbance correlation and does not use z.
    shared: u = alpha*z + r; a = beta*u + lambda*r + e, with unknown alpha,
      lambda and both residual scales. This is a joint Gaussian working model
      for (u,a)|z, rather than knowledge of the generator's latent mixture law.

    Parameters have fixed bounded priors. Zero intercepts, constant coefficients,
    iid intervals/axes and Gaussian working residuals are modeling assumptions.
    No observation noise calibration or generator parameters are supplied.
    """

    def __init__(self, *, mode: str = "independent", particleCount: int = 3000,
                 prior: ParameterPrior | None = None, resamplingThreshold: float = 0.5,
                 moveSteps: int = 8, seed: int = 1):
        if mode not in ("independent", "shared"):
            raise ValueError("mode must be independent or shared")
        if not isinstance(particleCount, int) or particleCount < 2:
            raise ValueError("particleCount must be an integer >= 2")
        if not 0 < resamplingThreshold <= 1:
            raise ValueError("resamplingThreshold must lie in (0, 1]")
        if not isinstance(moveSteps, int) or moveSteps < 1:
            raise ValueError("moveSteps must be a positive integer")
        self.mode = mode
        self.particleCount = particleCount
        self.prior = prior if prior is not None else ParameterPrior()
        self.resamplingThreshold = resamplingThreshold
        self.moveSteps = moveSteps
        self.rng = np.random.default_rng(seed)
        logScale = tuple(np.log(self.prior.noiseStddev))
        bounds = [self.prior.beta, logScale]
        self.parameterNames = ("beta", "logResidualStddev")
        if mode == "shared":
            bounds = [self.prior.beta, self.prior.inputResponse, self.prior.confoundingSlope,
                      logScale, logScale]
            self.parameterNames = ("beta", "inputResponse", "confoundingSlope",
                                   "logInputStddev", "logResidualStddev")
        self._bounds = np.array(bounds, dtype=float)
        self._particles = self.rng.uniform(self._bounds[:, 0], self._bounds[:, 1],
                                           size=(particleCount, len(bounds)))
        self._weights = np.full(particleCount, 1 / particleCount)
        # n, sum(a*a), sum(u*u), sum(z*z), sum(a*u), sum(a*z), sum(u*z)
        self._statistics = np.zeros(7)
        self._dimension = None
        self._count = 0
        self.history: list[ParameterEstimate] = []
        self._nextTime = 0.0

    @property
    def initialized(self) -> bool:
        return True

    @property
    def particles(self):
        """Marginal beta samples, matching the weights property."""
        return self._particles[:, 0].copy()

    @property
    def parameterParticles(self):
        """Joint samples; columns are listed in parameterNames."""
        return self._particles.copy()

    @property
    def weights(self):
        return self._weights.copy()

    def calculateStateEstimate(self) -> float:
        return float(self._weights @ self._particles[:, 0])

    @staticmethod
    def _summarize(observation):
        a, u, z = observation.arrays()
        return np.array([len(a), a @ a, u @ u, z @ z, a @ u, a @ z, u @ z], dtype=float)

    def _logLikelihood(self, particles, statistics):
        n, aa, uu, zz, au, az, uz = statistics
        beta = particles[:, 0]
        if self.mode == "independent":
            logScale = particles[:, 1]
            errorSquares = aa - 2 * beta * au + beta**2 * uu
            return -n * (logScale + 0.5 * np.log(2 * np.pi)) - 0.5 * np.maximum(errorSquares, 0) * np.exp(-2 * logScale)
        alpha, slope, logInputScale, logResidualScale = particles[:, 1:].T
        inputSquares = uu - 2 * alpha * uz + alpha**2 * zz
        # a - beta*u - lambda*(u-alpha*z)
        b, c = beta + slope, slope * alpha
        errorSquares = aa + b**2 * uu + c**2 * zz - 2 * b * au + 2 * c * az - 2 * b * c * uz
        return (-n * (logInputScale + logResidualScale + np.log(2 * np.pi))
                - 0.5 * np.maximum(inputSquares, 0) * np.exp(-2 * logInputScale)
                - 0.5 * np.maximum(errorSquares, 0) * np.exp(-2 * logResidualScale))

    def _resampleAndMove(self, proposalCholesky):
        positions = (self.rng.random() + np.arange(self.particleCount)) / self.particleCount
        cumulative = np.cumsum(self._weights)
        cumulative[-1] = 1
        self._particles = self._particles[np.searchsorted(cumulative, positions)].copy()
        self._weights.fill(1 / self.particleCount)
        current = self._logLikelihood(self._particles, self._statistics)
        accepted = 0
        for _ in range(self.moveSteps):
            proposal = self._particles + self.rng.normal(size=self._particles.shape) @ proposalCholesky.T
            inside = np.all((proposal >= self._bounds[:, 0]) & (proposal <= self._bounds[:, 1]), axis=1)
            proposed = np.full(self.particleCount, -np.inf)
            proposed[inside] = self._logLikelihood(proposal[inside], self._statistics)
            accept = np.log(self.rng.random(self.particleCount)) < proposed - current
            self._particles[accept] = proposal[accept]
            current[accept] = proposed[accept]
            accepted += np.count_nonzero(accept)
        return accepted / (self.moveSteps * self.particleCount)

    def update(self, measurement: ParameterObservation | None, context: None = None,
               dt: float = 1.0, *, time: float | None = None, predict: bool = True) -> float:
        """Consume observed acceleration/u/z. dt labels time; beta has no drift.

        Sufficient statistics contain only received observations. Histories record
        the weighted posterior before resampling and posterior-invariant moves.
        """
        positive(dt, "dt")
        time = self._nextTime if time is None else time
        positive(time, "time", allowZero=True)
        if measurement is None:
            self._nextTime = time + dt
            return self.calculateStateEstimate()
        statistics = self._summarize(measurement)
        if self._dimension is not None and statistics[0] != self._dimension:
            raise ValueError("Observation dimension must remain constant")
        with np.errstate(divide="ignore"):
            logWeights = np.log(self._weights) + self._logLikelihood(self._particles, statistics)
        normalizer = np.logaddexp.reduce(logWeights)
        if not np.isfinite(normalizer):
            raise ValueError("Non-finite likelihood; check observation scale")
        self._weights = np.exp(logWeights - normalizer)
        self._statistics += statistics
        self._dimension = statistics[0]
        self._count += 1
        mean = self.calculateStateEstimate()
        beta = self._particles[:, 0]
        stddev = float(np.sqrt(self._weights @ (beta - mean)**2))
        order = np.argsort(beta)
        indices = np.searchsorted(np.cumsum(self._weights[order]), (0.025, 0.975))
        lower, upper = beta[order[np.minimum(indices, self.particleCount - 1)]]
        ess = float(1 / (self._weights @ self._weights))
        resampled = ess < self.resamplingThreshold * self.particleCount
        acceptance = None
        if resampled:
            centered = self._particles - self._weights @ self._particles
            covariance = (centered * self._weights[:, None]).T @ centered
            covariance += np.diag(np.diff(self._bounds, axis=1).ravel()**2 * 1e-8)
            proposalCholesky = np.linalg.cholesky(covariance) * (2.38 / np.sqrt(len(self.parameterNames)))
            acceptance = self._resampleAndMove(proposalCholesky)
        self.history.append(ParameterEstimate(time, mean, stddev, float(lower), float(upper),
                                              ess, resampled, acceptance, self._count))
        self._nextTime = time + dt
        return mean
