import io
from pathlib import Path
import tempfile
import unittest
from contextlib import redirect_stdout

from controls import (
    ControlCommand, Coord, DataLogger, EstimatorDiagnostics, LogEntry,
    ReaderState, RFIDMeasurement, TagState,
)
from controls.main import main
from controls.Visualization import animateSimulation


class MotionAnimationTests(unittest.TestCase):
    def test_all_frames_including_missing_and_singular_covariance(self):
        logger = DataLogger()
        for i in range(3):
            estimate = TagState(Coord(2, 2)) if i else None
            diagnostics = EstimatorDiagnostics(0.25, 0, 0.04) if i == 1 else EstimatorDiagnostics(0, 0, 0)
            logger.log(LogEntry(i * 0.1, TagState(Coord(2, 2)), estimate, ReaderState(),
                                RFIDMeasurement(), ControlCommand(), diagnostics=diagnostics))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motion.html"
            animateSimulation(logger, path, title="EKF <test>")
            html = path.read_text()
            self.assertIn("EKF &lt;test&gt;", html)
            self.assertIn("nominal 95%", html)
            self.assertIn("frames = new Array(3)", html)
            self.assertEqual(html.count("data:image/png;base64,"), 3)
            self.assertIn('aria-label="Play"', html)
            self.assertIn('aria-label="Next frame"', html)
            self.assertNotIn("https://", html)
        with self.assertRaises(ValueError):
            animateSimulation(DataLogger(), "empty.html")
        with self.assertRaises(ValueError):
            animateSimulation(logger, "wrong.png")

    def test_cli_generates_ekf_motion_animation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario = root / "scenario.csv"
            scenario.write_text("time,x,y,vx,vy\n0,4,2,0,0\n0.1,4,2,0,0\n")
            animation = root / "ekf.html"
            with redirect_stdout(io.StringIO()):
                main(["--scenario", str(scenario), "--estimator", "ekf", "--seed", "42",
                      "--output", str(root / "ekf.csv"), "--animation", str(animation)])
            self.assertIn("frames = new Array(2)", animation.read_text())


if __name__ == "__main__":
    unittest.main()
