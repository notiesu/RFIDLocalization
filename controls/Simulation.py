"""Discrete-time feedback loop corresponding to ControlModel.jpg."""

from math import floor

from .Controller import Controller
from .DataLogger import DataLogger, LogEntry
from .Estimator import Estimator
from .ReaderPlant import ReaderPlant
from .RFIDSensor import RFIDSensor
from .ScenarioGenerator import ScenarioGenerator
from .Utils import ControlCommand, RFIDMeasurement, ReaderState, TagState


class Simulation:
    def __init__(
        self, scenario: ScenarioGenerator, sensor: RFIDSensor,
        estimator: Estimator[TagState, RFIDMeasurement, ReaderState],
        controller: Controller, plant: ReaderPlant, logger: DataLogger | None = None,
    ):
        self.scenario = scenario
        self.sensor = sensor
        self.estimator = estimator
        self.controller = controller
        self.plant = plant
        self.logger = logger if logger is not None else DataLogger()
        if self.logger.maxVelocity is None:
            self.logger.maxVelocity = controller.config.maxVelocity
        if self.logger.maxTurnRate is None:
            self.logger.maxTurnRate = controller.config.maxTurnRate
        self.timestep = scenario.scenario.dt
        if sensor.config.updateRate * self.timestep > 1.0 + 1e-12:
            raise ValueError("Simulation dt must be <= 1 / sensor updateRate")
        self._tick = 0
        self._measurementIndex = 0

    def step(self) -> LogEntry:
        trueTag = self.scenario.step()
        time = self._tick * self.timestep
        reader = self.plant.getState()
        due = time + 1e-12 >= self._measurementIndex / self.sensor.config.updateRate
        measurement = self.sensor.measure(trueTag, reader) if due else None
        if due:
            self._measurementIndex = floor((time + 1e-12) * self.sensor.config.updateRate) + 1
        # At t=0 the prior already describes t=0; do not advance it one tick early.
        estimate = self.estimator.update(
            measurement, reader, self.timestep, time=time, predict=self._tick != 0,
        )
        command = self.controller.compute(estimate, reader) if estimate is not None else ControlCommand()
        entry = LogEntry(time, trueTag, estimate, reader,
                         measurement if measurement is not None else RFIDMeasurement(uncertainty=self.sensor.config.rssiStdDev),
                         command, due, self.estimator.diagnostics,
                         self.scenario.scenario.events.get(self.scenario.index - 1, ""))
        # Every logged state and measurement describes t, before applying u[t].
        self.logger.log(entry)
        if self.scenario.index < len(self.scenario.scenario.states):
            self.plant.update(command, self.timestep)
        self._tick += 1
        return entry

    def run(self) -> DataLogger:
        while self.scenario.index < len(self.scenario.scenario.states):
            self.step()
        return self.logger
