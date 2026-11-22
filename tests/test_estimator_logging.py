import csv
import io
import math
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest

from controls import (
    Coord, ParticleFilterEstimator, EstimatorLogger, Particle, ReaderState, RFIDConfig,
    RFIDMeasurement, TagState,
)
import test_controls as helpers


class ParticleLoggingTests(unittest.TestCase):
    def test_logs_every_simulation_tick_and_matches_estimate(self):
        simulation = helpers.IntegrationTests.simulation(dt=0.02, duration=0.4, rate=10, dropout=0)
        simulation.run()
        snapshots = simulation.estimator.logger.entries
        self.assertEqual(len(snapshots), len(simulation.logger.entries))
        for snapshot, state in zip(snapshots, simulation.logger.entries):
            self.assertEqual(snapshot.time, state.time)
            self.assertEqual(snapshot.reader, state.reader)
            self.assertEqual(snapshot.estimatedTag, state.estimatedTag)
            self.assertEqual(snapshot.measurement is not None, state.measurementDue)
            self.assertEqual(len(snapshot.particles), 500)
            self.assertAlmostEqual(sum(p.weight for p in snapshot.particles), 1)
            self.assertAlmostEqual(sum(p.x * p.weight for p in snapshot.particles), state.estimatedTag.position.x)
            self.assertAlmostEqual(sum(p.y * p.weight for p in snapshot.particles), state.estimatedTag.position.y)

    def test_keeps_weighted_cloud_before_resampling_and_copies_inputs(self):
        estimator = ParticleFilterEstimator(RFIDConfig(bearingStdDev=0.1), particleCount=2, accelerationStdDev=0,
                              resamplingThreshold=0.9)
        estimator.particleField = [Particle(TagState(Coord(5, 0)), 0.5),
                                   Particle(TagState(Coord(0, 5)), 0.5)]
        reader = ReaderState()
        measurement = RFIDMeasurement(-40 - 20 * math.log10(5), True, 1, 0)
        estimate = estimator.update(measurement, reader, 0.1)
        snapshot = estimator.logger.entries[0]
        self.assertTrue(snapshot.resampled)
        self.assertLess(snapshot.effectiveSampleSize, 1.01)
        self.assertGreater(snapshot.particles[0].weight, 0.999)
        self.assertTrue(all(p.weight == 0.5 for p in estimator.particleField))
        estimator.particleField[0].state.position.x = 999
        reader.position.x = 999
        measurement.Pr = 999
        estimate.position.x = 999
        self.assertEqual(snapshot.particles[0].x, 5)
        self.assertEqual(snapshot.reader.position.x, 0)
        self.assertNotEqual(snapshot.measurement.Pr, 999)
        self.assertNotEqual(snapshot.estimatedTag.position.x, 999)

    def test_empty_clouds_dropouts_acquisition_and_csv(self):
        estimator = ParticleFilterEstimator(RFIDConfig(), particleCount=2)
        estimator.update(None, ReaderState(), 0.1)
        estimator.update(RFIDMeasurement(), ReaderState(), 0.1)
        estimator.update(RFIDMeasurement(-54, True, 2, 0), ReaderState(), 0.1)
        entries = estimator.logger.entries
        self.assertEqual([e.time for e in entries], [0, 0.1, 0.2])
        self.assertEqual([len(e.particles) for e in entries], [0, 0, 2])
        self.assertIsNone(entries[0].estimatedTag)
        self.assertIsNotNone(entries[2].estimatedTag)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "particles.csv"
            with redirect_stdout(io.StringIO()):
                estimator.logger.exportCSV(path)
            with path.open(newline="") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 4)
            self.assertTrue(all(len(row) == 22 and None not in row for row in rows))
            self.assertEqual(rows[0]["particle_index"], "")
            self.assertEqual(rows[0]["measurement_due"], "0")
            self.assertEqual(rows[1]["measurement_due"], "1")
            self.assertEqual(rows[1]["valid"], "0")
            self.assertEqual(rows[2]["valid"], "1")
        estimator.logger.clear()
        self.assertEqual(estimator.logger.entries, [])

    def test_logging_does_not_change_filter_results(self):
        enabled = helpers.IntegrationTests.simulation(duration=2)
        disabled = helpers.IntegrationTests.simulation(duration=2)
        disabled.estimator.logger = EstimatorLogger(enabled=False)
        self.assertEqual(enabled.run().entries, disabled.run().entries)
        self.assertEqual(disabled.estimator.logger.entries, [])
        self.assertTrue(enabled.estimator.logger.entries)

    def test_time_zero_with_prior_is_logged_without_prediction(self):
        simulation = helpers.IntegrationTests.simulation(duration=0, prior=TagState(Coord(4, 2), 1, 0), dropout=1)
        before = simulation.estimator.calculateStateEstimate()
        simulation.run()
        snapshot = simulation.estimator.logger.entries[0]
        self.assertEqual(snapshot.time, 0)
        self.assertEqual(snapshot.estimatedTag, before)


class AnimationTests(unittest.TestCase):
    def test_offline_animation_including_empty_cloud(self):
        try:
            from controls.Visualization import animateParticles
            import matplotlib
        except ImportError:
            self.skipTest("Matplotlib is optional")
        estimator = ParticleFilterEstimator(RFIDConfig(), particleCount=2)
        estimator.update(None, ReaderState(), 0.1)
        estimator.update(RFIDMeasurement(-54, True, 2, 0), ReaderState(), 0.1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "particles.html"
            animateParticles(estimator.logger, path)
            html = path.read_text()
            self.assertIn("data:image/png;base64,", html)
            self.assertIn('type="range"', html)
            self.assertIn('aria-label="Play"', html)
            self.assertIn('aria-label="Next frame"', html)
            self.assertNotIn("https://", html)
            self.assertNotIn("http://", html)
            self.assertIn('frames = new Array(2)', html)
        with self.assertRaises(ValueError):
            animateParticles(EstimatorLogger(), "empty.html")
        simulation = helpers.IntegrationTests.simulation(duration=0)
        simulation.run()
        with self.assertRaises(ValueError):
            animateParticles(estimator.logger, "mismatch.html", simulation.logger)


if __name__ == "__main__":
    unittest.main()
