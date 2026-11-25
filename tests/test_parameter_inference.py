import ast
import inspect
import json
from pathlib import Path
import unittest

import numpy as np

from controls.Estimator import Estimator
from controls.ParameterParticleFilterEstimator import (
    ParameterPrior, ParameterObservation, ParameterParticleFilterEstimator,
)
from controls.InstrumentalVariableEstimator import InstrumentalVariableEstimator


class ParameterInferenceTests(unittest.TestCase):
    def test_joint_likelihood_matches_direct_observed_data_density(self):
        f = ParameterParticleFilterEstimator(mode="shared")
        obs = ParameterObservation((0.7, -0.3), (1.2, -0.4), (0.4, -0.1))
        a, u, z = obs.arrays()
        parameters = np.array([[0.5, 0.8, 0.2, np.log(0.7), np.log(0.15)],
                               [-0.2, -0.6, 0.9, np.log(1.2), np.log(0.8)]])
        for row, actual in zip(parameters, f._logLikelihood(parameters, f._summarize(obs))):
            beta, alpha, slope, lr, le = row
            residual = u - alpha * z
            expected = np.sum(-lr - 0.5 * np.log(2 * np.pi) - 0.5 * residual**2 * np.exp(-2*lr))
            expected += np.sum(-le - 0.5 * np.log(2 * np.pi)
                               - 0.5 * (a - beta*u - slope*residual)**2 * np.exp(-2*le))
            self.assertAlmostEqual(actual, expected, places=10)

    def test_independent_mode_ignores_z_but_shared_mode_uses_it(self):
        first = ParameterObservation((0.7,), (1.2,), (0.4,))
        second = ParameterObservation((0.7,), (1.2,), (2.0,))
        for mode in ("independent", "shared"):
            f = ParameterParticleFilterEstimator(mode=mode)
            params = np.array([[0.5, -1.]]) if mode == "independent" else np.array([[0.5, 1., 0.2, -0.2, -1.]])
            values = [f._logLikelihood(params, f._summarize(obs))[0] for obs in (first, second)]
            if mode == "independent":
                self.assertEqual(*values)
            else:
                self.assertGreater(abs(values[0] - values[1]), 0.1)

    def test_unknown_noise_posterior_matches_dense_grid_and_rejuvenates(self):
        f = ParameterParticleFilterEstimator(particleCount=6000, seed=8)
        rng = np.random.default_rng(14)
        for _ in range(70):
            u = rng.normal(size=2)
            a = 0.7*u + rng.normal(0, 0.4, 2)
            f.update(ParameterObservation(tuple(a), tuple(u), tuple(rng.normal(size=2))))
        beta, logScale = np.meshgrid(np.linspace(0.4, 1., 500), np.linspace(-1.4, -0.4, 500))
        particles = np.column_stack((beta.ravel(), logScale.ravel()))
        logPosterior = f._logLikelihood(particles, f._statistics)
        weights = np.exp(logPosterior - np.max(logPosterior))
        weights /= weights.sum()
        exactMean = weights @ particles[:, 0]
        self.assertAlmostEqual(f.calculateStateEstimate(), exactMean, delta=0.008)
        self.assertAlmostEqual(f.history[-1].stddev,
                               np.sqrt(weights @ (particles[:, 0]-exactMean)**2), delta=0.005)
        self.assertTrue(any(h.resampled and h.moveAcceptanceRate > 0 for h in f.history))
        self.assertGreater(len(np.unique(f.particles)), 0.5*f.particleCount)
        # Scale is learned, rather than supplied as true observation/process noise.
        estimatedScale = f.weights @ np.exp(f.parameterParticles[:, 1])
        self.assertAlmostEqual(estimatedScale, 0.4, delta=0.08)

    def test_shared_inference_learns_input_and_noise_without_nuisance_truth(self):
        rng = np.random.default_rng(11)
        n = 700
        z, w, nu = rng.normal(size=(3, n, 2))
        u = 1.3*z + 0.8*w + 0.2*nu
        a = 0.7*u + 0.3*w + rng.normal(0, 0.08, (n, 2))
        f = ParameterParticleFilterEstimator(mode="shared", particleCount=6000, seed=4)
        iv = InstrumentalVariableEstimator()
        for ai, ui, zi in zip(a, u, z):
            obs = ParameterObservation(tuple(ai), tuple(ui), tuple(zi))
            f.update(obs)
            iv.update(obs)
        estimates = dict(zip(f.parameterNames, f.weights @ f.parameterParticles))
        self.assertAlmostEqual(estimates["beta"], iv.calculateStateEstimate(), delta=0.025)
        self.assertAlmostEqual(estimates["beta"], 0.7, delta=0.04)
        self.assertAlmostEqual(estimates["inputResponse"], 1.3, delta=0.06)
        self.assertGreater(estimates["confoundingSlope"], 0.1)

    def test_iv_recovers_beta_while_ordinary_regression_is_confounded(self):
        rng = np.random.default_rng(4)
        n = 10000
        z, w, nu = rng.normal(size=(3, n, 2))
        u = z + 0.8*w + 0.2*nu
        a = 0.5*u + 0.3*w
        iv = InstrumentalVariableEstimator()
        for ai, ui, zi in zip(a, u, z):
            iv.update(ParameterObservation(tuple(ai), tuple(ui), tuple(zi)))
        expected = np.sum(z*a)/np.sum(z*u)
        self.assertAlmostEqual(iv.calculateStateEstimate(), expected)
        self.assertAlmostEqual(expected, 0.5, delta=0.01)
        self.assertGreater(np.sum(u*a)/np.sum(u*u), 0.6)

    def test_reproducibility_no_observation_and_sequential_boundary(self):
        a = ParameterParticleFilterEstimator(seed=9, particleCount=500)
        b = ParameterParticleFilterEstimator(seed=9, particleCount=500)
        self.assertIsInstance(a, Estimator)
        self.assertTrue(a.initialized)
        original = a.parameterParticles
        a.update(None, dt=0.1)
        np.testing.assert_array_equal(a.parameterParticles, original)
        obs = ParameterObservation((0.7,), (1.2,), (0.4,))
        for _ in range(5):
            self.assertEqual(a.update(obs, time=1), b.update(obs, time=1))
        prefix = list(a.history)
        b.update(ParameterObservation((999.,), (-123.,), (22.,)))
        self.assertEqual(a.history, prefix)
        self.assertEqual(b.history[:-1], prefix)
        external = a.parameterParticles
        external[:] = 100
        self.assertTrue(np.all(a.particles <= 3))

    def test_no_truth_configuration_or_truth_access(self):
        signature = inspect.signature(ParameterParticleFilterEstimator)
        self.assertEqual(set(signature.parameters),
                         {"mode", "particleCount", "prior", "resamplingThreshold", "moveSteps", "seed"})
        self.assertEqual(set(ParameterObservation.__dataclass_fields__), {"acceleration", "inputs", "instruments"})
        tree = ast.parse(inspect.getsource(inspect.getmodule(ParameterParticleFilterEstimator)))
        self.assertFalse(any(isinstance(n, ast.Attribute) and n.attr in ("truth", "latentDraws", "scenario") for n in ast.walk(tree)))
        with self.assertRaises(TypeError):
            ParameterParticleFilterEstimator(observationStddev=0.05)
        with self.assertRaises(TypeError):
            ParameterParticleFilterEstimator(disturbance={"gamma": 0.3})

    def test_notebook_inference_runs_without_generator_or_truth_in_scope(self):
        notebookPath = Path(__file__).resolve().parents[1] / "notebooks" / "compare_beta_estimators.ipynb"
        notebook = json.loads(notebookPath.read_text())
        functions = []
        for cell in notebook["cells"]:
            source = "".join(cell["source"])
            if cell["cell_type"] == "code" and "def infer_beta(" in source:
                functions = [node for node in ast.parse(source).body if isinstance(node, ast.FunctionDef)]
        self.assertEqual(len(functions), 1)
        # Execute the actual notebook helper with only an explicit inference allowlist.
        # No generator, result, truth, sensor-noise calibration, or true beta exists.
        namespace = {"np": np, "ParameterParticleFilterEstimator": ParameterParticleFilterEstimator,
                     "InstrumentalVariableEstimator": InstrumentalVariableEstimator}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(notebookPath), "exec"), namespace)
        observations = [ParameterObservation((0.7,), (1.2,), (0.4,))] * 4
        run = namespace["infer_beta"](observations, dt=0.1, particle_count=300, seed=5, prior=ParameterPrior())
        self.assertEqual(len(run["estimators"]), 3)
        self.assertTrue(all(len(values) == 4 for values in run["estimates"].values()))
        # No truth metadata reads anywhere in the comparison notebook's code.
        for cell in notebook["cells"]:
            if cell["cell_type"] == "code":
                source = "\n".join(line for line in "".join(cell["source"]).splitlines() if not line.startswith("%"))
                self.assertFalse(any(isinstance(n, ast.Attribute) and n.attr in ("truth", "latentDraws")
                                     for n in ast.walk(ast.parse(source))))

    def test_invalid_data_priors_and_unidentified_iv(self):
        for overrides in ({"mode": "bad"}, {"particleCount": 1}, {"moveSteps": 0}, {"resamplingThreshold": 2}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                ParameterParticleFilterEstimator(**overrides)
        for overrides in ({"beta": (1, 0)}, {"noiseStddev": (0, 1)}, {"inputResponse": (0, float("inf"))}):
            with self.assertRaises(ValueError):
                ParameterPrior(**overrides)
        for invalid in (ParameterObservation((), (), ()), ParameterObservation((1,), (1, 2), (1,)),
                        ParameterObservation((float("nan"),), (1,), (1,))):
            with self.assertRaises(ValueError):
                invalid.arrays()
        iv = InstrumentalVariableEstimator()
        self.assertIsNone(iv.update(ParameterObservation((1,), (2,), (0,))))
        with self.assertRaises(RuntimeError):
            iv.calculateStateEstimate()
        with self.assertRaises(ValueError):
            iv.update(ParameterObservation((1, 1), (1, 1), (1, 1)))


if __name__ == "__main__":
    unittest.main()
