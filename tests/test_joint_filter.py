import ast
from copy import deepcopy
import inspect
import json
from pathlib import Path
import unittest

import numpy as np

from controls import (Estimator, JointParticleFilterEstimator, JointFilterContext, RFIDConfig,
                      RFIDSensor, RFIDMeasurement, ReaderState, TagState, Coord)


class JointFilterTests(unittest.TestCase):
    def make(self, **kwargs):
        return JointParticleFilterEstimator(RFIDConfig(rssiStdDev=0.5, bearingStdDev=0.02),
                                             parameterCount=48, stateParticleCount=32, **kwargs)

    def initialize(self, f):
        measurement = RFIDMeasurement(-49., True, 0.5, 0.6)
        f.update(measurement, JointFilterContext(ReaderState()), 0.2, time=0, predict=False)

    def test_acquisition_and_input_requirements(self):
        f = self.make()
        self.assertIsInstance(f, Estimator)
        before = f.parameterParticles
        self.assertIsNone(f.update(None, JointFilterContext(ReaderState()), 0.2))
        self.assertFalse(f.initialized)
        np.testing.assert_array_equal(before, f.parameterParticles)
        with self.assertRaises(RuntimeError):
            f.calculateStateEstimate()
        self.initialize(f)
        self.assertEqual(len(f.history), 1)
        self.assertEqual(len(f._records), 0)  # No prediction/double assimilation at t=0.
        with self.assertRaises(ValueError):
            f.update(None, JointFilterContext(ReaderState()), 0.2)

    def test_forecast_moments_and_no_mutation(self):
        f = self.make(mode="independent")
        self.initialize(f)
        f._states[:] = [2, 3, 0.2, -0.1]
        f._parameters[:] = [0.7, np.log(0.4)]
        f._stateWeights.fill(1 / f.stateParticleCount)
        f._parameterWeights.fill(1 / f.parameterCount)
        previous, rng = f.stateParticles, deepcopy(f.rng.bit_generator.state)
        prediction = f.predictNextState(JointFilterContext(ReaderState(), (1., -2.), (0.3, 0.4)), 0.2)
        np.testing.assert_allclose([prediction.mean.position.x, prediction.mean.position.y,
                                    prediction.mean.xdot, prediction.mean.ydot], [2.04, 2.98, 0.34, -0.38])
        np.testing.assert_allclose(prediction.covariance, np.diag([0, 0, 0.0064, 0.0064]), atol=1e-12)
        np.testing.assert_array_equal(previous, f.stateParticles)
        self.assertEqual(rng, f.rng.bit_generator.state)

    def test_euler_and_shared_input_likelihood(self):
        f = self.make(mode="shared")
        parameters = np.array([[0.5, 1., 0.3, np.log(0.8), np.log(1e-8)]])
        states = np.tile([2., 3., 0.2, -0.1], (1, 32, 1))
        weights = np.full((1, 32), 1/32)
        context = JointFilterContext(ReaderState(), (1.2, -0.7), (0.4, -0.2))
        after, _, likelihood = f._advance(parameters, states.copy(), weights, (None, context, 0.2, True))
        acceleration = 0.5*np.array(context.inputs) + 0.3*(np.array(context.inputs)-np.array(context.instruments))
        np.testing.assert_allclose(after[0, :, :2], np.tile([2.04, 2.98], (32, 1)))
        np.testing.assert_allclose(after[0, :, 2:], np.tile(np.array([0.2, -0.1])+0.2*acceleration, (32, 1)), atol=1e-7)
        expected = -2*np.log(0.8*np.sqrt(2*np.pi)) - 0.5*((1.2-.4)**2+(-.7+.2)**2)/.8**2
        self.assertAlmostEqual(likelihood[0], expected)

    def test_inner_resampling_keeps_rows_separate(self):
        f = self.make()
        parameters = np.array([[0., 0.], [1., 0.]])
        states = np.arange(2*32*4, dtype=float).reshape(2, 32, 4)
        weights = np.zeros((2, 32))
        weights[0, 5] = 1
        weights[1, 19] = 1
        after, weights, _ = f._advance(parameters, states.copy(), weights,
                                      (None, JointFilterContext(ReaderState()), 0.2, False))
        np.testing.assert_array_equal(after[0], np.tile(states[0, 5], (32, 1)))
        np.testing.assert_array_equal(after[1], np.tile(states[1, 19], (32, 1)))
        np.testing.assert_allclose(weights, 1/32)

    def test_predictions_do_not_see_current_measurement_and_records_are_copied(self):
        first, second = self.make(seed=4), self.make(seed=4)
        self.initialize(first)
        self.initialize(second)
        context = JointFilterContext(ReaderState(), (0.8, -0.2), (0.3, 0.1))
        reading = RFIDMeasurement(-48.8, True, 0.5, 0.61)
        first.update(reading, context, 0.2)
        second.update(RFIDMeasurement(-50., True, 0.5, 0.58), context, 0.2)
        self.assertEqual(first.lastPrediction.mean, second.lastPrediction.mean)
        np.testing.assert_array_equal(first.lastPrediction.covariance, second.lastPrediction.covariance)
        reading.Pr = 999
        context.reader.position.x = 999
        self.assertEqual(first._records[0][0].Pr, -48.8)
        self.assertEqual(first._records[0][1].reader.position.x, 0)

    def test_rfid_tracking_dropouts_replays_and_static_parameter_support(self):
        config = RFIDConfig(rssiStdDev=0.3, bearingStdDev=0.01)
        sensor = RFIDSensor(config, seed=5)
        f = JointParticleFilterEstimator(config, parameterCount=96, stateParticleCount=64, seed=6)
        reader = ReaderState()
        estimates, truth = [], []
        for i in range(25):
            state = TagState(Coord(3+0.05*i, 2+0.02*i), 0.25, 0.1)
            measurement = sensor.measure(state, reader) if i not in (10, 11, 12) else None
            context = JointFilterContext(reader, (0., 0.), (0., 0.)) if i else JointFilterContext(reader)
            estimate = f.update(measurement, context, 0.2, time=i*.2, predict=i>0)
            if estimate:
                estimates.append([estimate.position.x, estimate.position.y])
                truth.append([state.position.x, state.position.y])
        rmse = np.sqrt(np.mean(np.sum((np.array(estimates)-truth)**2, axis=1)))
        self.assertLess(rmse, 0.4)
        self.assertTrue(any(h.resampled for h in f.history))
        self.assertTrue(all(np.isfinite(h.beta) and h.lower95 <= h.beta <= h.upper95 for h in f.history))
        self.assertTrue(np.all(np.isfinite(f.stateParticles)))
        self.assertAlmostEqual(f.parameterWeights.sum(), 1)
        self.assertGreaterEqual(f.diagnostics.covarianceXX, 0)
        self.assertTrue(np.all(np.abs(f.parameterParticles[:, 0]) <= 3))

    def test_boundary_and_validation(self):
        signature = inspect.signature(JointParticleFilterEstimator)
        self.assertNotIn("initialState", signature.parameters)
        self.assertNotIn("beta", signature.parameters)
        self.assertNotIn("observationStddev", signature.parameters)
        tree = ast.parse(inspect.getsource(inspect.getmodule(JointParticleFilterEstimator)))
        self.assertFalse(any(isinstance(n, ast.Attribute) and n.attr in ("truth", "latentDraws", "scenario") for n in ast.walk(tree)))
        for kwargs in ({"mode": "bad"}, {"stateParticleCount": 1}, {"initialVelocityStddev": 0}):
            with self.assertRaises(ValueError):
                JointParticleFilterEstimator(RFIDConfig(), **kwargs)
        f = self.make()
        with self.assertRaises(ValueError):
            f.update(RFIDMeasurement(float("nan"), True, 0.5, 0), JointFilterContext(ReaderState()), 0.2)

    def test_notebook_helper_has_no_true_states_or_motion_parameters_in_scope(self):
        path = Path(__file__).resolve().parents[1] / "notebooks" / "joint_state_parameter_estimation.ipynb"
        notebook = json.loads(path.read_text())
        functions = []
        for cell in notebook["cells"]:
            if cell["cell_type"] == "code" and "def run_joint(" in "".join(cell["source"]):
                functions = [n for n in ast.parse("".join(cell["source"])).body if isinstance(n, ast.FunctionDef)]
        self.assertEqual(len(functions), 1)
        namespace = {"np": np, "JointParticleFilterEstimator": JointParticleFilterEstimator}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), "exec"), namespace)
        records = [(i*.2, RFIDMeasurement(-49., True, 0.5, 0.6),
                    JointFilterContext(ReaderState(), (0.1, 0.2), (0.2, 0.1))) for i in range(5)]
        from controls import ParameterPrior
        run = namespace["run_joint"](records, sensor_config=RFIDConfig(), dt=.2, mode="shared",
                                      parameter_count=12, state_particle_count=16, prior=ParameterPrior(), seed=1)
        self.assertEqual(run["states"].shape, (5, 4))
        self.assertEqual(len(run["beta"]), 5)
        self.assertTrue(np.all(np.isfinite(run["beta"])))


if __name__ == "__main__":
    unittest.main()
