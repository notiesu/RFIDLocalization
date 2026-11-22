from copy import deepcopy
from math import isnan
import unittest

from controls import (
    Controller, ControllerConfig, Coord, Estimator, ParticleFilterEstimator,
    ReaderPlant, ReaderState, RFIDConfig, RFIDMeasurement, RFIDSensor,
    Scenario, ScenarioGenerator, Simulation, TagState,
)


class FixedPositionEstimator(Estimator[TagState, RFIDMeasurement, ReaderState]):
    """A test estimator with no particle state or logger."""

    def __init__(self):
        self.estimate = None
        self.calls = []

    @property
    def initialized(self):
        return self.estimate is not None

    def calculateStateEstimate(self):
        if not self.initialized:
            raise RuntimeError("No estimate")
        return deepcopy(self.estimate)

    def update(self, measurement, context, dt, *, time=None, predict=True):
        self.calls.append((time, predict))
        if measurement is not None and measurement.isValid:
            self.estimate = TagState(Coord(4, 2))
        return self.calculateStateEstimate() if self.initialized else None


class EstimatorInterfaceTests(unittest.TestCase):
    def test_base_is_abstract_and_particle_filter_implements_it(self):
        with self.assertRaises(TypeError):
            Estimator()
        estimator = ParticleFilterEstimator(RFIDConfig(), particleCount=2)
        self.assertIsInstance(estimator, Estimator)
        self.assertFalse(estimator.initialized)
        self.assertIs(estimator.diagnostics, estimator.logger.latestDiagnostics)

    def test_simulation_accepts_an_estimator_without_particle_logging(self):
        estimator = FixedPositionEstimator()
        with self.assertRaises(RuntimeError):
            estimator.calculateStateEstimate()
        scenario = Scenario(0.1, [TagState(Coord(4, 2)) for _ in range(3)])
        simulation = Simulation(
            ScenarioGenerator(scenario), RFIDSensor(RFIDConfig(dropoutProbability=0)),
            estimator, Controller(ControllerConfig()), ReaderPlant(ReaderState()),
        )
        logger = simulation.run()
        self.assertTrue(estimator.initialized)
        self.assertFalse(hasattr(estimator, "logger"))
        self.assertEqual(estimator.calls, [(0, False), (0.1, True), (0.2, True)])
        self.assertEqual(logger.stats()["positionRMSE"], 0)
        self.assertTrue(all(isnan(e.diagnostics.positionUncertainty) for e in logger.entries))


if __name__ == "__main__":
    unittest.main()
