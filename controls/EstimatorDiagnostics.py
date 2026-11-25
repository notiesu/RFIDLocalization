"""Shared tracking diagnostics, independent of any estimator's logging backend."""

from dataclasses import dataclass
from math import nan, sqrt


@dataclass(frozen=True)
class EstimatorDiagnostics:
    covarianceXX: float = nan
    covarianceXY: float = nan
    covarianceYY: float = nan
    effectiveSampleSize: float = 0.0
    resampled: bool = False
    innovationRSSI: float = nan
    innovationBearing: float = nan
    normalizedInnovationSquared: float = nan
    innovationGateThreshold: float = nan
    innovationGateMode: str = ""
    innovationOutlier: bool | None = None
    measurementAccepted: bool | None = None
    measurementRejectionReason: str = ""

    @property
    def positionUncertainty(self) -> float:
        """RMS distance from the estimated position mean, in meters."""
        return sqrt(self.covarianceXX + self.covarianceYY)
