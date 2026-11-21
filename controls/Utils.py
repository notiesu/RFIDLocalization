"""Shared C++ structs, represented as Python dataclasses.

Distances are meters, time is seconds, and angles are radians.
Names intentionally match cpp/include/Utils.h.
"""

from dataclasses import dataclass, field
from math import atan2, isfinite, sin, cos


def wrapAngle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def positive(value: float, name: str, *, allowZero: bool = False) -> None:
    if not isfinite(value) or (value < 0 if allowZero else value <= 0):
        raise ValueError(f"{name} must be finite and {'nonnegative' if allowZero else 'positive'}")


@dataclass
class Coord:
    x: float = 0.0
    y: float = 0.0


@dataclass
class TagState:
    position: Coord = field(default_factory=Coord)
    xdot: float = 0.0
    ydot: float = 0.0


@dataclass
class ReaderState:
    position: Coord = field(default_factory=Coord)
    heading: float = 0.0
    turnSpeed: float = 0.0
    velocity: float = 0.0


@dataclass
class RFIDMeasurement:
    Pr: float = 0.0
    isValid: bool = False
    uncertainty: float = 0.0  # RSSI standard deviation in dB
    bearing: float = 0.0     # Relative to the reader heading


@dataclass
class ControlCommand:
    turnRate: float = 0.0  # rad/s, as used by ReaderPlant.cpp
    velocity: float = 0.0


@dataclass
class Particle:
    state: TagState
    weight: float


@dataclass
class Scenario:
    dt: float
    states: list[TagState]

    def __post_init__(self) -> None:
        positive(self.dt, "dt")
