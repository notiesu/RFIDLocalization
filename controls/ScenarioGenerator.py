"""Read external ground-truth scenarios and iterate their stored states."""

import csv
from copy import deepcopy
from math import isclose, isfinite
from pathlib import Path

from .Utils import Coord, Scenario, TagState


def loadScenarioCSV(filename: str | Path) -> Scenario:
    """Load time,x,y,vx,vy in seconds, meters, and meters/second.

    The fixed-step simulator requires >=2 rows, starting at zero with uniform
    positive time spacing. Extra named columns are ignored. No interpolation or
    velocity inference is performed; the external generator supplies all states.
    """
    columns = ("time", "x", "y", "vx", "vy")
    times = []
    states = []
    try:
        with open(filename, newline="", encoding="utf-8-sig") as file:
            reader = csv.DictReader(file, strict=True)
            headers = reader.fieldnames or []
            if not set(columns).issubset(headers) or len(headers) != len(set(headers)):
                raise ValueError(f"{filename}: CSV must have unique headers including {','.join(columns)}")
            for row in reader:
                if None in row:
                    raise ValueError(f"{filename}: row {reader.line_num} has too many fields")
                try:
                    time, x, y, vx, vy = (float(row[name]) for name in columns)
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"{filename}: row {reader.line_num} requires numeric time,x,y,vx,vy") from exc
                if not all(isfinite(value) for value in (time, x, y, vx, vy)):
                    raise ValueError(f"{filename}: row {reader.line_num} contains non-finite values")
                times.append(time)
                states.append(TagState(Coord(x, y), vx, vy))
    except csv.Error as exc:
        raise ValueError(f"{filename}: invalid CSV: {exc}") from exc
    if len(states) < 2:
        raise ValueError(f"{filename}: scenario needs at least two data rows to determine dt")
    if times[0] != 0:
        raise ValueError(f"{filename}: timestamps must start at 0 seconds")
    dt = times[1] - times[0]
    if dt <= 0 or any(
        current <= previous or not isclose(current, index * dt, rel_tol=1e-7, abs_tol=1e-9)
        for index, (previous, current) in enumerate(zip(times, times[1:]), start=1)
    ):
        raise ValueError(f"{filename}: timestamps must increase at a uniform positive timestep")
    return Scenario(dt, states)


class ScenarioGenerator:
    def __init__(self, scenario: Scenario):
        self.scenario = deepcopy(scenario)
        self.index = 0

    def step(self) -> TagState:
        if self.index >= len(self.scenario.states):
            raise StopIteration("Scenario exhausted")
        state = deepcopy(self.scenario.states[self.index])
        self.index += 1
        return state

    def reset(self) -> None:
        self.index = 0
