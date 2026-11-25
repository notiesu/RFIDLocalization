import math
from pathlib import Path
import tempfile
import unittest

from controls import loadScenarioCSV
from controls.scenarios.randomwalk import generateScenarios


class RandomWalkTests(unittest.TestCase):
    def test_csvs_are_reproducible_and_velocities_match_outgoing_intervals(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = generateScenarios(Path(directory))
            self.assertEqual([p.name for p in paths], ["gaussian.csv", "student_t.csv"])
            before = [p.read_bytes() for p in paths]
            generateScenarios(Path(directory))
            self.assertEqual(before, [p.read_bytes() for p in paths])
            for path in paths:
                scenario = loadScenarioCSV(path)
                self.assertEqual(len(scenario.states), 201)
                self.assertEqual((scenario.states[0].position.x, scenario.states[0].position.y), (5, 5))
                for current, following in zip(scenario.states, scenario.states[1:]):
                    self.assertAlmostEqual(following.position.x - current.position.x, current.xdot * scenario.dt)
                    self.assertAlmostEqual(following.position.y - current.position.y, current.ydot * scenario.dt)
            generateScenarios(Path(directory), seed=43)
            self.assertNotEqual(before, [p.read_bytes() for p in paths])

    def test_matched_increment_scale_and_heavier_student_t_tails(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = generateScenarios(Path(directory), duration=1000)
            tails = []
            for path in paths:
                scenario = loadScenarioCSV(path)
                values = [v * math.sqrt(scenario.dt) / 0.3 for s in scenario.states[:-1] for v in (s.xdot, s.ydot)]
                mean = sum(values) / len(values)
                variance = sum((value - mean) ** 2 for value in values) / len(values)
                self.assertLess(abs(mean), 0.1)
                self.assertTrue(0.6 < variance < 1.6, (path.name, variance))
                tails.append(sum(abs(value) > 4 for value in values))
            self.assertGreater(tails[1], 10 * max(tails[0], 1))

    def test_rejects_invalid_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            for kwargs in ({"dt": 0}, {"duration": 0.01}, {"diffusion": -1}, {"duration": math.nan}):
                with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                    generateScenarios(Path(directory), **kwargs)


if __name__ == "__main__":
    unittest.main()
