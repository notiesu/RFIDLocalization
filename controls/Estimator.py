"""Common interface for sequential estimators, independent of their algorithm."""

from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from .EstimatorDiagnostics import EstimatorDiagnostics


StateT = TypeVar("StateT")
MeasurementT = TypeVar("MeasurementT")
ContextT = TypeVar("ContextT")


class Estimator(ABC, Generic[StateT, MeasurementT, ContextT]):
    """Subclasses choose the state, measurement, and known-context types.

    update() consumes one tick and returns an estimate, or None before acquisition.
    A missing measurement means prediction only. Explicit time aligns simulation
    logs; predict=False means the current prior already describes that timestamp.
    """

    @property
    @abstractmethod
    def initialized(self) -> bool:
        """Whether a state estimate is available."""
        raise NotImplementedError

    @abstractmethod
    def update(
        self, measurement: MeasurementT | None, context: ContextT, dt: float, /,
        *, time: float | None = None, predict: bool = True,
    ) -> StateT | None:
        """Advance the estimator and optionally incorporate a fresh observation."""
        raise NotImplementedError

    @abstractmethod
    def calculateStateEstimate(self) -> StateT:
        """Return the current estimate; raise RuntimeError before initialization."""
        raise NotImplementedError

    @property
    def diagnostics(self) -> EstimatorDiagnostics:
        """Optional tracking diagnostics; unavailable values default to NaN."""
        return EstimatorDiagnostics()
