"""Proportional pursuit controller filling the original empty C++ stub."""

from copy import deepcopy
from dataclasses import dataclass
from math import atan2, cos, hypot

from .Utils import ControlCommand, ReaderState, TagState, positive, wrapAngle


@dataclass
class ControllerConfig:
    Kv: float = 0.5
    Kw: float = 2.0
    maxVelocity: float = 2.0
    maxTurnRate: float = 1.5

    def __post_init__(self) -> None:
        for name in ("Kv", "Kw", "maxVelocity", "maxTurnRate"):
            positive(getattr(self, name), name, allowZero=True)


class Controller:
    def __init__(self, config: ControllerConfig):
        self.config = deepcopy(config)

    def compute(self, estimatedTag: TagState, reader: ReaderState) -> ControlCommand:
        dx = estimatedTag.position.x - reader.position.x
        dy = estimatedTag.position.y - reader.position.y
        distance = hypot(dx, dy)
        if distance <= 1e-12:
            return ControlCommand()
        error = wrapAngle(atan2(dy, dx) - reader.heading)
        c = self.config
        turnRate = max(-c.maxTurnRate, min(c.maxTurnRate, c.Kw * error))
        # Slow down while turning; turn in place when the tag is behind us.
        velocity = min(c.maxVelocity, c.Kv * distance) * max(0.0, cos(error))
        return ControlCommand(turnRate, velocity)
