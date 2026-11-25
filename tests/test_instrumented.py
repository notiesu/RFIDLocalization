import csv
import json
from pathlib import Path
import tempfile
import unittest

from controls.ScenarioGenerator import loadScenarioCSV
from controls.Utils import Coord, TagState
from controls.scenarios.instruments import (
    InstrumentSample, generateInstruments, instrumentTimestep,
    loadInstrumentCSV, writeInstrumentCSV,
)
from controls.scenarios.instrumented import generateTrajectory, writeTrajectoryCSV


class InstrumentedTrajectoryTests(unittest.TestCase):
    def test_equations_and_shared_confounder(self):
        instruments = generateInstruments(duration=2)
        result = generateTrajectory(instruments, initialState=TagState(Coord(1, 2), 3, 4))
        c = result.truth["coefficients"]
        dt = result.scenario.dt
        for i, (z, u, latent) in enumerate(zip(instruments, result.inputs, result.truth["latentDraws"])):
            before, after = result.scenario.states[i:i + 2]
            self.assertAlmostEqual(after.position.x, before.position.x + before.xdot * dt)
            self.assertAlmostEqual(after.position.y, before.position.y + before.ydot * dt)
            for axis, zi, vi, vnext in zip(range(2), (z.z_x, z.z_y),
                                          (before.xdot, before.ydot), (after.xdot, after.ydot)):
                w, nu = latent["w"][axis], latent["nu"][axis]
                self.assertAlmostEqual(u[axis], c["alpha"] * zi + c["delta"] * w + nu)
                self.assertAlmostEqual(vnext, vi + (c["beta"] * u[axis] + c["gamma"] * w) * dt)

    def test_seed_separation_and_fixed_distribution(self):
        z = generateInstruments(duration=2, seed=42)
        a = generateTrajectory(z, seed=1, distributionSeed=7)
        self.assertEqual(a, generateTrajectory(z, seed=1, distributionSeed=7))
        b = generateTrajectory(z, seed=2, distributionSeed=7)
        self.assertEqual(a.truth["disturbance"], b.truth["disturbance"])
        self.assertNotEqual(a.truth["latentDraws"], b.truth["latentDraws"])
        self.assertEqual(a.instruments, b.instruments)
        law = a.truth["disturbance"]
        p, means, sigmas = law["probability"], law["means"], law["stddevs"]
        self.assertAlmostEqual(p * means[0] + (1 - p) * means[1], 0)
        self.assertAlmostEqual(sum(q * (m*m + s*s) for q, m, s in
                                   zip((p, 1-p), means, sigmas)), 1)
        # Numeric seeds shared by modules must not duplicate their RNG streams.
        sameSeed = generateTrajectory(z, seed=42)
        self.assertNotEqual([s.z_x for s in z], [s["nu"][0] / 0.2 for s in sameSeed.truth["latentDraws"]])

    def test_csv_roundtrip_and_existing_loader(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            z = generateInstruments(duration=2)
            writeInstrumentCSV(root / "z.csv", z)
            self.assertEqual(z, loadInstrumentCSV(root / "z.csv"))
            result = generateTrajectory(z)
            path, truth = writeTrajectoryCSV(root / "trajectory.csv", result)
            self.assertEqual(loadScenarioCSV(path), result.scenario)
            self.assertEqual(len(result.scenario.states), len(z) + 1)
            self.assertAlmostEqual((len(result.scenario.states) - 1) * result.scenario.dt, 2)
            with path.open() as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(rows[-1]["u_x"], "")
            self.assertEqual(rows[-1]["z_y"], "")
            self.assertNotIn("w_x", rows[0])
            self.assertEqual(len(json.loads(truth.read_text())["latentDraws"]), len(z))

    def test_invalid_inputs(self):
        for kwargs in ({"dt": 0}, {"duration": 0.1}, {"duration": 1.05}, {"stddev": float("nan")}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                generateInstruments(**kwargs)
        for samples in ([], [InstrumentSample(0, 1, 1)],
                        [InstrumentSample(1, 1, 1), InstrumentSample(2, 1, 1)],
                        [InstrumentSample(0, 1, 1), InstrumentSample(0, 1, 1)],
                        [InstrumentSample(0, 1, 1), InstrumentSample(0.1, float("inf"), 1)],
                        [InstrumentSample(0, 1, 1), InstrumentSample(0.1, 1, 1), InstrumentSample(0.3, 1, 1)]):
            with self.subTest(samples=samples), self.assertRaises(ValueError):
                instrumentTimestep(samples)
        z = generateInstruments(duration=2)
        for kwargs in ({"beta": float("inf")}, {"inputNoiseStddev": -1},
                       {"initialState": TagState(Coord(float("nan"), 0))}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                generateTrajectory(z, **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.csv"
            for text in ("time,z_x\n0,1\n", "time,z_x,z_y\n0,no,1\n0.1,1,1\n",
                         "time,z_x,z_y\n0,1,1,1\n0.1,1,1\n"):
                path.write_text(text)
                with self.assertRaises(ValueError):
                    loadInstrumentCSV(path)


if __name__ == "__main__":
    unittest.main()
