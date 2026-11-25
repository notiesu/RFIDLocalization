"""Running zero-intercept IV estimate of a common scalar beta across axes."""

from dataclasses import dataclass

import numpy as np

from .Estimator import Estimator
from .ParameterParticleFilterEstimator import ParameterObservation
from .Utils import positive


@dataclass(frozen=True)
class IVEstimate:
    time: float
    beta: float | None
    observationCount: int
    firstStageCoefficient: float | None


class InstrumentalVariableEstimator(Estimator[float, ParameterObservation, None]):
    """Solve sum_t,axis z*(a-beta*u)=0, pooling the common beta.

    Assumes zero-mean structural disturbances and valid paired instruments.
    Does not consume nuisance parameters or latent noise. Near-zero first-stage
    cross-moments return None; finite estimates can still suffer weak instruments.
    """

    def __init__(self):
        self._za = self._zu = self._zz = self._uu = 0.0
        self._dimension = None
        self._count = 0
        self._nextTime = 0.0
        self.history: list[IVEstimate] = []

    @property
    def initialized(self) -> bool:
        return self._zz > 0 and abs(self._zu) > 1e-10 * np.sqrt(self._zz * self._uu)

    def calculateStateEstimate(self) -> float:
        if not self.initialized:
            raise RuntimeError("IV needs nonzero instrument/input cross-moments")
        return self._za / self._zu

    def update(self, measurement: ParameterObservation | None, context: None = None,
               dt: float = 1.0, *, time: float | None = None, predict: bool = True) -> float | None:
        positive(dt, "dt")
        time = self._nextTime if time is None else time
        positive(time, "time", allowZero=True)
        if measurement is not None:
            a, u, z = measurement.arrays()
            if self._dimension is not None and len(a) != self._dimension:
                raise ValueError("Observation dimension must remain constant")
            self._dimension = len(a)
            self._za += float(z @ a)
            self._zu += float(z @ u)
            self._zz += float(z @ z)
            self._uu += float(u @ u)
            self._count += 1
        beta = self.calculateStateEstimate() if self.initialized else None
        if measurement is not None:
            self.history.append(IVEstimate(time, beta, self._count,
                                          self._zu / self._zz if self._zz > 0 else None))
        self._nextTime = time + dt
        return beta
