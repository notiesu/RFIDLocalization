import csv
import io
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from controls import Coord, TagState, loadScenarioCSV
from controls.main import main


class ScenarioCSVTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "ground-truth.csv"

    def write(self, content):
        self.path.write_text(content, encoding="utf-8")
        return self.path

    def test_loads_external_states_and_infers_dt(self):
        scenario = loadScenarioCSV(self.write(
            "time,x,y,vx,vy\n0,4,2,0.5,0.15\n0.05,4.025,2.0075,0.5,0.15\n0.1,4.05,2.015,0.5,0.15\n"
        ))
        self.assertEqual(scenario.dt, 0.05)
        self.assertEqual(len(scenario.states), 3)
        self.assertEqual(scenario.states[1], TagState(Coord(4.025, 2.0075), 0.5, 0.15))

    def test_accepts_bom_reordered_headers_and_extra_metadata(self):
        scenario = loadScenarioCSV(self.write(
            '\ufefflabel,vy,time,y,vx,x\n"start, here",0,0,2,1,4\nend,0,0.1,2,1,4.1\n'
        ))
        self.assertEqual(scenario.states[0], TagState(Coord(4, 2), 1, 0))

    def test_rejects_invalid_input(self):
        cases = {
            "empty": "",
            "no rows": "time,x,y,vx,vy\n",
            "one row": "time,x,y,vx,vy\n0,1,2,0,0\n",
            "missing column": "time,x,y,vx\n0,1,2,0\n0.1,1,2,0\n",
            "duplicate header": "time,x,y,vx,vy,x\n0,1,2,0,0,1\n0.1,1,2,0,0,1\n",
            "missing value": "time,x,y,vx,vy\n0,1,2,0\n0.1,1,2,0,0\n",
            "extra field": "time,x,y,vx,vy\n0,1,2,0,0,oops\n0.1,1,2,0,0\n",
            "nonnumeric": "time,x,y,vx,vy\n0,whoops,2,0,0\n0.1,1,2,0,0\n",
            "NaN": "time,x,y,vx,vy\n0,nan,2,0,0\n0.1,1,2,0,0\n",
            "infinity": "time,x,y,vx,vy\n0,1,2,0,0\ninf,1,2,0,0\n",
            "nonzero start": "time,x,y,vx,vy\n1,1,2,0,0\n1.1,1,2,0,0\n",
            "duplicate time": "time,x,y,vx,vy\n0,1,2,0,0\n0,1,2,0,0\n",
            "reversed time": "time,x,y,vx,vy\n0,1,2,0,0\n-0.1,1,2,0,0\n",
            "irregular time": "time,x,y,vx,vy\n0,1,2,0,0\n0.1,1,2,0,0\n0.25,1,2,0,0\n",
            "malformed quoting": 'time,x,y,vx,vy\n0,"unterminated,2,0,0\n',
        }
        for name, content in cases.items():
            with self.subTest(name=name), self.assertRaises(ValueError):
                loadScenarioCSV(self.write(content))

    def test_cli_uses_csv_through_last_sample(self):
        self.write("time,x,y,vx,vy\n0,4,2,0.5,0\n0.05,4.025,2,0.5,0\n0.1,4.05,2,0.5,0\n")
        output = Path(self.directory.name) / "results.csv"
        with redirect_stdout(io.StringIO()):
            status = main(["--scenario", str(self.path), "--output", str(output), "--particles", "10"])
        self.assertEqual(status, 0)
        with output.open(newline="") as file:
            rows = list(csv.DictReader(file))
        self.assertEqual([float(row["time"]) for row in rows], [0, 0.05, 0.1])
        self.assertEqual([float(row["true_x"]) for row in rows], [4, 4.025, 4.05])
        self.assertTrue(all(float(row["true_vx"]) == 0.5 for row in rows))

    def test_cli_requires_file_and_reports_errors_without_traceback(self):
        for args in ([], ["--scenario", str(self.path)], ["--scenario", "line"]):
            with self.subTest(args=args), redirect_stderr(io.StringIO()) as errors:
                with self.assertRaises(SystemExit) as exc:
                    main(args)
                self.assertEqual(exc.exception.code, 2)
                self.assertNotIn("Traceback", errors.getvalue())

    def test_cli_preserves_input_when_output_path_matches(self):
        content = "time,x,y,vx,vy\n0,4,2,0,0\n0.1,4,2,0,0\n"
        self.write(content)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
            main(["--scenario", str(self.path), "--output", str(self.path)])
        self.assertEqual(exc.exception.code, 2)
        self.assertEqual(self.path.read_text(), content)


if __name__ == "__main__":
    unittest.main()
