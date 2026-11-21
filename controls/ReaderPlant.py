"""Port of the midpoint-heading motion update in ReaderPlant.cpp."""

from copy import deepcopy
from math import cos, sin

from .Utils import ControlCommand, ReaderState, positive, wrapAngle


class ReaderPlant:
    def __init__(self, initialState: ReaderState):
        self.state = deepcopy(initialState)

    def getState(self) -> ReaderState:
        # C++ returns a value, so callers must not mutate the live plant state.
        return deepcopy(self.state)

    def update(self, command: ControlCommand, dt: float) -> None:
        positive(dt, "dt")
        omega = command.turnRate
        midHeading = self.state.heading + omega * dt / 2.0
        self.state.position.x += command.velocity * cos(midHeading) * dt
        self.state.position.y += command.velocity * sin(midHeading) * dt
        self.state.heading = wrapAngle(self.state.heading + omega * dt)
        self.state.velocity = command.velocity
        self.state.turnSpeed = omega
