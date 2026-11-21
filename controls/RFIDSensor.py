"""Port of RFIDSensor.cpp; timing is handled by Simulation."""

from copy import deepcopy
from dataclasses import dataclass
from math import atan2, hypot, isfinite, log10
from random import Random

from .Utils import RFIDMeasurement, ReaderState, TagState, positive, wrapAngle


@dataclass
class RFIDConfig:
    referenceRSSI: float = -40.0
    referenceDistance: float = 1.0
    pathLossExponent: float = 2.0
    rssiStdDev: float = 2.0
    bearingStdDev: float = 0.1
    maxRange: float = 10.0
    minRSSI: float = -80.0
    updateRate: float = 10.0
    dropoutProbability: float = 0.05

    def __post_init__(self) -> None:
        for name in ("referenceDistance", "pathLossExponent", "maxRange", "updateRate"):
            positive(getattr(self, name), name)
        for name in ("rssiStdDev", "bearingStdDev"):
            positive(getattr(self, name), name, allowZero=True)
        if not isfinite(self.referenceRSSI) or not isfinite(self.minRSSI):
            raise ValueError("RSSI levels must be finite")
        if not 0 <= self.dropoutProbability <= 1:
            raise ValueError("dropoutProbability must lie between 0 and 1")


class RFIDSensor:
    def __init__(self, config: RFIDConfig, seed: int = 0):
        self.config = deepcopy(config)
        self.rng = Random(seed)

    @staticmethod
    def wrapAngle(angle: float) -> float:
        return wrapAngle(angle)

    def measure(self, trueTag: TagState, reader: ReaderState) -> RFIDMeasurement:
        dx = trueTag.position.x - reader.position.x
        dy = trueTag.position.y - reader.position.y
        distance = hypot(dx, dy)
        c = self.config
        if distance > c.maxRange or self.rng.random() < c.dropoutProbability:
            return RFIDMeasurement(0.0, False, c.rssiStdDev, 0.0)
        d = max(distance, c.referenceDistance)
        expectedRSSI = c.referenceRSSI - 10.0 * c.pathLossExponent * log10(d / c.referenceDistance)
        measuredRSSI = expectedRSSI + self.rng.gauss(0.0, c.rssiStdDev)
        if measuredRSSI < c.minRSSI:
            return RFIDMeasurement(0.0, False, c.rssiStdDev, 0.0)
        trueBearing = atan2(dy, dx) - reader.heading
        bearing = wrapAngle(trueBearing + self.rng.gauss(0.0, c.bearingStdDev))
        return RFIDMeasurement(measuredRSSI, True, c.rssiStdDev, bearing)
