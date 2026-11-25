import io
import math
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import numpy as np

from controls import (
    Controller, ControllerConfig, Coord, EKFEstimator, Estimator, ReaderPlant,
    ReaderState, RFIDConfig, RFIDMeasurement, RFIDSensor, ScenarioGenerator,
    Simulation, TagState, loadScenarioCSV,
)
from controls.main import main


class EKFTests(unittest.TestCase):
    def test_waits_for_acquisition_and_initializes_from_relative_bearing(self):
        config = RFIDConfig(dropoutProbability=0, rssiStdDev=0, bearingStdDev=0)
        estimator = EKFEstimator(config)
        self.assertIsInstance(estimator, Estimator)
        reader = ReaderState(Coord(1, 2), heading=0.7)
        self.assertIsNone(estimator.update(None, reader, 0.1))
        self.assertIsNone(estimator.update(RFIDMeasurement(), reader, 0.1))
        with self.assertRaises(RuntimeError):
            estimator.calculateStateEstimate()
        truth = TagState(Coord(4, 6))
        estimate = estimator.update(RFIDSensor(config).measure(truth, reader), reader, 0.1)
        self.assertAlmostEqual(estimate.position.x, 4)
        self.assertAlmostEqual(estimate.position.y, 6)
        self.assertEqual((estimate.xdot, estimate.ydot), (0, 0))
        self.assertTrue(np.all(np.linalg.eigvalsh(estimator.covariance) >= 0))

    def test_constant_velocity_prediction_and_process_covariance(self):
        estimator = EKFEstimator(RFIDConfig(), initialState=TagState(Coord(1, 2), 3, -1))
        estimate = estimator.update(None, ReaderState(), 0.2)
        self.assertAlmostEqual(estimate.position.x, 1.6)
        self.assertAlmostEqual(estimate.position.y, 1.8)
        self.assertAlmostEqual(estimator.covariance[0, 0], 1.0401)
        self.assertAlmostEqual(estimator.covariance[0, 2], 0.201)
        self.assertAlmostEqual(estimator.covariance[2, 2], 1.01)
        self.assertEqual(estimator.diagnostics.covarianceXX, estimator.covariance[0, 0])
        estimate.position.x = 999
        self.assertNotEqual(estimator.calculateStateEstimate().position.x, 999)

    def test_measurement_jacobian_matches_finite_differences(self):
        estimator = EKFEstimator(RFIDConfig())
        reader = ReaderState(Coord(-1, 0.5), heading=0.3)
        state = np.array([3., 4., 0.1, -0.2])
        _, jacobian = estimator._measurementModel(state, reader)
        numerical = np.zeros((2, 4))
        for column in range(4):
            delta = np.zeros(4)
            delta[column] = 1e-6
            upper, _ = estimator._measurementModel(state + delta, reader)
            lower, _ = estimator._measurementModel(state - delta, reader)
            numerical[:, column] = (upper - lower) / 2e-6
        np.testing.assert_allclose(jacobian, numerical, atol=1e-7)

    def test_correction_improves_range_and_reduces_uncertainty(self):
        estimator = EKFEstimator(RFIDConfig(), initialState=TagState(Coord(4, 0)))
        before = np.trace(estimator.covariance[:2, :2])
        reading = RFIDMeasurement(-40 - 20 * math.log10(5), True, 2, 0)
        estimate = estimator.update(reading, ReaderState(), 0.1, predict=False)
        self.assertTrue(4 < estimate.position.x < 5)
        self.assertLess(np.trace(estimator.covariance[:2, :2]), before)

    def test_wrapped_bearing_and_degenerate_geometry(self):
        estimator = EKFEstimator(RFIDConfig(), initialState=TagState(Coord(-5, 0.05)))
        reading = RFIDMeasurement(-40 - 20 * math.log10(5), True, 2, -math.pi + 0.01)
        before = estimator.state.copy()
        estimator.update(reading, ReaderState(), 0.1, predict=False)
        self.assertLess(np.linalg.norm(estimator.state[:2] - before[:2]), 0.2)
        near = EKFEstimator(RFIDConfig(), initialState=TagState(Coord(0.5, 0.5)))
        _, jacobian = near._measurementModel(near.state, ReaderState())
        np.testing.assert_equal(jacobian[0], np.zeros(4))
        near.update(RFIDMeasurement(-80, True, 2, math.pi / 4), ReaderState(), 0.1, predict=False)
        np.testing.assert_allclose(near.state[:2], [0.5, 0.5])
        coincident = EKFEstimator(RFIDConfig(), initialState=TagState())
        coincident.update(RFIDMeasurement(-40, True, 0, 0), ReaderState(), 0.1)
        self.assertTrue(np.all(np.isfinite(coincident.covariance)))

    def test_invalid_priors_and_measurements(self):
        for covariance in (np.eye(3), np.diag([-1, 1, 1, 1]), np.full((4, 4), math.nan),
                           np.eye(4) + np.diag([1, 1, 1], k=1)):
            with self.subTest(covariance=covariance), self.assertRaises(ValueError):
                EKFEstimator(RFIDConfig(), initialState=TagState(), initialCovariance=covariance)
        with self.assertRaises(ValueError):
            EKFEstimator(RFIDConfig(), initialCovariance=np.eye(4))
        with self.assertRaises(ValueError):
            EKFEstimator(RFIDConfig()).update(RFIDMeasurement(math.nan, True, 2, 0), ReaderState(), 0.1)

    def test_closed_loop_covariance_stays_finite_and_psd(self):
        for name in ("straight", "turning", "abrupt_maneuver"):
            with self.subTest(name=name):
                scenario = loadScenarioCSV(Path(__file__).resolve().parents[1] / "scenarios" / "assignment" / f"{name}.csv")
                config = RFIDConfig()
                estimator = EKFEstimator(config)
                simulation = Simulation(ScenarioGenerator(scenario), RFIDSensor(config, 42), estimator,
                                        Controller(ControllerConfig()), ReaderPlant(ReaderState()))
                while simulation.scenario.index < len(scenario.states):
                    simulation.step()
                    if estimator.initialized:
                        self.assertTrue(np.all(np.isfinite(estimator.state)))
                        np.testing.assert_allclose(estimator.covariance, estimator.covariance.T, atol=1e-12)
                        self.assertGreaterEqual(np.linalg.eigvalsh(estimator.covariance).min(), -1e-10)

    def test_fixed_reader_tracks_assignment_paths(self):
        # Check accuracy away from the RSSI plateau and bearing singularity;
        # pursuing through the tag is an EKF linearization stress case.
        for name in ("straight", "turning", "abrupt_maneuver"):
            with self.subTest(name=name):
                scenario = loadScenarioCSV(Path(__file__).resolve().parents[1] / "scenarios" / "assignment" / f"{name}.csv")
                config = RFIDConfig()
                simulation = Simulation(
                    ScenarioGenerator(scenario), RFIDSensor(config, 42), EKFEstimator(config),
                    Controller(ControllerConfig(maxVelocity=0, maxTurnRate=0)), ReaderPlant(ReaderState()),
                )
                self.assertLess(simulation.run().stats()["positionRMSE"], 0.7)

    def test_cli_ekf_and_incompatible_particle_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            scenario = Path(directory) / "scenario.csv"
            scenario.write_text("time,x,y,vx,vy\n0,4,2,0,0\n0.1,4,2,0,0\n")
            output = Path(directory) / "ekf.csv"
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--scenario", str(scenario), "--estimator", "ekf", "--output", str(output)]), 0)
            self.assertTrue(output.with_suffix(".summary.json").exists())
            for option in ("--particle-log", "--particle-animation"):
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    main(["--scenario", str(scenario), "--estimator", "ekf", option, "particles.html"])
                self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
