import csv
import io
import math
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest

from controls import (
    Controller, ControllerConfig, ControlCommand, Coord, DataLogger, ParticleFilterEstimator,
    LogEntry, Particle, ReaderPlant, ReaderState, RFIDConfig, RFIDMeasurement,
    RFIDSensor, Scenario, ScenarioGenerator, Simulation, TagState,
)


def makeTestScenario(name, duration, dt):
    """Synthetic trajectories used only as numerical test fixtures."""
    states = []
    for i in range(math.floor(duration / dt + 1e-10) + 1):
        t = i * dt
        if name == "circle":
            angle = 0.15 * t
            states.append(TagState(Coord(4 + 3 * math.cos(angle), 3 * math.sin(angle)),
                                   -0.45 * math.sin(angle), 0.45 * math.cos(angle)))
        else:
            states.append(TagState(Coord(4 + 0.5 * t, 2 + 0.15 * t), 0.5, 0.15))
    return Scenario(dt, states)


class PortTests(unittest.TestCase):
    def test_sensor_equations_and_relative_bearing(self):
        config = RFIDConfig(rssiStdDev=0, bearingStdDev=0, dropoutProbability=0)
        sensor = RFIDSensor(config)
        reading = sensor.measure(TagState(Coord(3, 4)), ReaderState(heading=0.7))
        self.assertTrue(reading.isValid)
        self.assertAlmostEqual(reading.Pr, -40 - 20 * math.log10(5))
        self.assertAlmostEqual(reading.bearing, math.atan2(4, 3) - 0.7)
        self.assertEqual(sensor.measure(TagState(Coord(0.25, 0)), ReaderState()).Pr, -40)

    def test_sensor_detection_boundaries(self):
        config = RFIDConfig(rssiStdDev=0, bearingStdDev=0, dropoutProbability=0)
        self.assertTrue(RFIDSensor(config).measure(TagState(Coord(10, 0)), ReaderState()).isValid)
        self.assertFalse(RFIDSensor(config).measure(TagState(Coord(10.01, 0)), ReaderState()).isValid)
        config.dropoutProbability = 1
        self.assertFalse(RFIDSensor(config).measure(TagState(), ReaderState()).isValid)
        config.dropoutProbability = 0
        config.minRSSI = -39
        self.assertFalse(RFIDSensor(config).measure(TagState(), ReaderState()).isValid)

    def test_midpoint_motion_and_value_semantics(self):
        initial = ReaderState(Coord(1, 2), 0.3)
        plant = ReaderPlant(initial)
        plant.update(ControlCommand(turnRate=0.4, velocity=2), 0.5)
        result = plant.getState()
        self.assertAlmostEqual(result.position.x, 1 + math.cos(0.4))
        self.assertAlmostEqual(result.position.y, 2 + math.sin(0.4))
        self.assertAlmostEqual(result.heading, 0.5)
        self.assertEqual(result.velocity, 2)
        self.assertEqual(result.turnSpeed, 0.4)
        result.position.x = 99
        self.assertNotEqual(plant.getState().position.x, 99)
        self.assertEqual(initial.position, Coord(1, 2))

    def test_heading_wrap(self):
        plant = ReaderPlant(ReaderState(heading=math.pi - 0.1))
        plant.update(ControlCommand(turnRate=1), 0.2)
        self.assertAlmostEqual(plant.getState().heading, -math.pi + 0.1)

    def test_scenario_exhaustion_reset_and_copy(self):
        source = Scenario(0.1, [TagState(Coord(1, 2))])
        generator = ScenarioGenerator(source)
        source.states[0].position.x = 99
        state = generator.step()
        self.assertEqual(state.position.x, 1)
        state.position.x = 42
        with self.assertRaises(StopIteration):
            generator.step()
        generator.reset()
        self.assertEqual(generator.step().position.x, 1)

    def test_logger_csv_metrics_and_snapshot(self):
        logger = DataLogger()
        entry = LogEntry(0, TagState(Coord(3, 4)), TagState(), ReaderState(),
                         RFIDMeasurement(-40, True, 2, 0), ControlCommand(0.2, 1))
        logger.log(entry)
        entry.trueTag.position.x = 999
        self.assertEqual(logger.stats()["positionRMSE"], 5)
        self.assertEqual(logger.stats()["meanTrackingDistance"], 5)
        self.assertEqual(logger.stats()["validRate"], 100)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "log.csv"
            with redirect_stdout(io.StringIO()):
                logger.exportCSV(path)
            with path.open(newline="") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows[0]), 41)
            self.assertEqual(float(rows[0]["true_x"]), 3)
            self.assertEqual(float(rows[0]["command_turnRate"]), 0.2)
            self.assertEqual(float(rows[0]["estimation_error"]), 5)
        logger.clear()
        self.assertEqual(logger.stats()["samples"], 0)


class NewComponentTests(unittest.TestCase):
    def test_controller_limits_turning_and_zero_distance(self):
        controller = Controller(ControllerConfig())
        ahead = controller.compute(TagState(Coord(100, 0)), ReaderState())
        self.assertEqual(ahead, ControlCommand(0, 2))
        behind = controller.compute(TagState(Coord(-10, 1)), ReaderState())
        self.assertEqual(behind.turnRate, 1.5)
        self.assertEqual(behind.velocity, 0)
        self.assertEqual(controller.compute(TagState(), ReaderState()), ControlCommand())
        wrapped = controller.compute(TagState(Coord(-10, -0.01)), ReaderState(heading=math.pi - 0.01))
        self.assertLess(abs(wrapped.turnRate), 0.1)

    def test_particle_prediction_during_dropout(self):
        estimator = ParticleFilterEstimator(RFIDConfig(), particleCount=2, accelerationStdDev=0)
        estimator.initializeParticles(TagState(Coord(1, 2), 3, -1), 0, 0)
        estimate = estimator.update(RFIDMeasurement(), ReaderState(), 0.5)
        self.assertEqual(estimate, TagState(Coord(2.5, 1.5), 3, -1))
        self.assertAlmostEqual(estimator.calculateEffectiveSampleSize(), 2)
        self.assertIsNone(ParticleFilterEstimator(RFIDConfig()).update(None, ReaderState(), 0.1))

    def test_measurement_likelihood_favors_correct_particle(self):
        config = RFIDConfig(rssiStdDev=1, bearingStdDev=0.1)
        estimator = ParticleFilterEstimator(config, particleCount=2)
        estimator.particleField = [Particle(TagState(Coord(5, 0)), 0.5),
                                   Particle(TagState(Coord(0, 5)), 0.5)]
        estimator.updateWeights(RFIDMeasurement(-40 - 20 * math.log10(5), True, 1, 0), ReaderState())
        self.assertGreater(estimator.particleField[0].weight, 0.999)
        self.assertAlmostEqual(sum(p.weight for p in estimator.particleField), 1)
        estimator.resampleParticles()
        self.assertTrue(all(p.state.position == Coord(5, 0) for p in estimator.particleField))
        estimator.particleField[0].state.position.x = 99
        self.assertEqual(estimator.particleField[1].state.position.x, 5)

    def test_bearing_likelihood_crosses_pi_boundary(self):
        estimator = ParticleFilterEstimator(RFIDConfig(bearingStdDev=0.1), particleCount=2)
        estimator.particleField = [Particle(TagState(Coord(-5, 0.01)), 0.5),
                                   Particle(TagState(Coord(5, 0)), 0.5)]
        estimator.updateWeights(RFIDMeasurement(-40 - 20 * math.log10(5), True, 2, -math.pi + 0.002),
                                ReaderState())
        self.assertGreater(estimator.particleField[0].weight, 0.999)

    def test_invalid_settings(self):
        for kwargs in ({"updateRate": 0}, {"dropoutProbability": 1.1},
                       {"rssiStdDev": -1}, {"referenceDistance": math.nan}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                RFIDConfig(**kwargs)
        with self.assertRaises(ValueError):
            ParticleFilterEstimator(RFIDConfig(), particleCount=1)
        with self.assertRaises(ValueError):
            Scenario(0, [])


class IntegrationTests(unittest.TestCase):
    @staticmethod
    def simulation(dt=0.1, duration=2, rate=10, dropout=0.05, prior=None, scenario="line"):
        config = RFIDConfig(updateRate=rate, dropoutProbability=dropout)
        return Simulation(
            ScenarioGenerator(makeTestScenario(scenario, duration, dt)), RFIDSensor(config, 42),
            ParticleFilterEstimator(config, particleCount=500, seed=43, initialState=prior),
            Controller(ControllerConfig()), ReaderPlant(ReaderState()),
        )

    def test_sampling_and_no_duplicate_assimilation(self):
        simulation = self.simulation(dt=0.02, duration=1, rate=10, dropout=0)
        logger = simulation.run()
        self.assertEqual(len(logger.entries), 51)
        self.assertEqual(logger.stats()["measurementAttempts"], 11)
        self.assertEqual(logger.stats()["validRate"], 100)
        for i, entry in enumerate(logger.entries):
            self.assertEqual(entry.measurementDue, i % 5 == 0)
            self.assertEqual(entry.measurement.isValid, entry.measurementDue)

    def test_noninteger_sensor_clock(self):
        simulation = self.simulation(dt=0.03, duration=1, rate=10, dropout=0)
        times = [e.time for e in simulation.run().entries if e.measurementDue]
        self.assertEqual(len(times), 10)
        for i, time in enumerate(times):
            self.assertGreaterEqual(time + 1e-12, i / 10)
            self.assertLess(time - i / 10, 0.03 + 1e-12)
        with self.assertRaises(ValueError):
            self.simulation(dt=0.2, rate=10)

    def test_no_acquisition_has_no_fabricated_estimate(self):
        simulation = self.simulation(dropout=1)
        logger = simulation.run()
        self.assertTrue(all(e.estimatedTag is None for e in logger.entries))
        self.assertEqual(simulation.plant.getState(), ReaderState())
        self.assertEqual(logger.stats()["estimatedSamples"], 0)
        self.assertEqual(logger.stats()["validRate"], 0)

    def test_timestamp_alignment_and_final_plant_state(self):
        simulation = self.simulation(dt=0.1, duration=0.2, dropout=0)
        logger = simulation.run()
        self.assertEqual(logger.entries[0].reader, ReaderState())
        expected = ReaderPlant(logger.entries[0].reader)
        expected.update(logger.entries[0].command, 0.1)
        self.assertEqual(logger.entries[1].reader, expected.getState())
        self.assertEqual(simulation.plant.getState(), logger.entries[-1].reader)
        before = simulation.plant.getState()
        with self.assertRaises(StopIteration):
            simulation.step()
        self.assertEqual(before, simulation.plant.getState())

    def test_prior_is_not_advanced_at_time_zero(self):
        prior = TagState(Coord(4, 2), 1, 0)
        simulation = self.simulation(duration=0, prior=prior, dropout=1)
        initial = simulation.estimator.calculateStateEstimate()
        self.assertEqual(simulation.run().entries[0].estimatedTag, initial)

    def test_reproducible_closed_loop_tracking(self):
        simulation = self.simulation(duration=30)
        logger = simulation.run()
        stats = logger.stats()
        self.assertLess(stats["positionRMSE"], 0.6)
        self.assertLess(stats["meanTrackingDistance"], 2)
        self.assertEqual(stats["estimatedSamples"], stats["samples"])
        repeat = self.simulation(duration=30).run()
        self.assertEqual(logger.entries, repeat.entries)
        for entry in logger.entries:
            self.assertLessEqual(abs(entry.command.turnRate), 1.5)
            self.assertTrue(0 <= entry.command.velocity <= 2)

    def test_curved_motion_tracking(self):
        logger = self.simulation(dt=0.05, duration=30, scenario="circle").run()
        self.assertLess(logger.stats()["positionRMSE"], 0.8)
        self.assertLess(logger.stats()["meanTrackingDistance"], 2)


if __name__ == "__main__":
    unittest.main()
