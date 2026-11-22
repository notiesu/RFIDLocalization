import csv
import io
import json
import math
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from controls import (
    ControlCommand, Coord, DataLogger, EstimatorDiagnostics, EstimatorLogger,
    LogEntry, Particle, ReaderState, RFIDMeasurement, TagState,
)
from controls.main import main


def entry(time, error=0.2, *, due=True, valid=True, event="", command=None, uncertainty=1):
    return LogEntry(
        time, TagState(), TagState(Coord(error, 0)), ReaderState(),
        RFIDMeasurement(-50, valid, 2, 0), command or ControlCommand(), due,
        EstimatorDiagnostics(uncertainty ** 2, 0, 0, 2, False), event,
    )


class LoggingMetricTests(unittest.TestCase):
    def test_weighted_covariance_even_without_particle_history(self):
        particles = [Particle(TagState(Coord(0, 0)), 0.25), Particle(TagState(Coord(2, 4)), 0.75)]
        logger = EstimatorLogger(enabled=False)
        logger.log(0, particles, TagState(Coord(1.5, 3)), ReaderState(), None, 1.6, False)
        diagnostics = logger.latestDiagnostics
        self.assertEqual(logger.entries, [])
        self.assertAlmostEqual(diagnostics.covarianceXX, 0.75)
        self.assertAlmostEqual(diagnostics.covarianceXY, 1.5)
        self.assertAlmostEqual(diagnostics.covarianceYY, 3)
        self.assertAlmostEqual(diagnostics.positionUncertainty, math.sqrt(3.75))

    def test_time_weighted_availability_uncertainty_and_command_limits(self):
        logger = DataLogger(maxVelocity=2, maxTurnRate=1.5)
        logger.log(entry(0, command=ControlCommand(0, 2), uncertainty=1))
        logger.log(entry(0.5, due=False, valid=False, command=ControlCommand(-1.5, 1), uncertainty=2))
        logger.log(entry(1.5, valid=False, command=ControlCommand(0, 2), uncertainty=3))
        logger.log(entry(2, command=ControlCommand(1.5, 2), uncertainty=99))
        stats = logger.stats()
        self.assertAlmostEqual(stats["validRate"], 100 * 2 / 3)
        self.assertEqual(stats["validTimeRate"], 75)
        self.assertEqual(stats["speedLimitUsage"], 50)
        self.assertEqual(stats["turnRateLimitUsage"], 50)
        self.assertEqual(stats["meanPositionUncertainty"], 2)
        self.assertEqual(stats["maxPositionUncertainty"], 99)
        self.assertEqual(stats["uncertaintyTimeCoverage"], 100)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.csv"
            with redirect_stdout(io.StringIO()):
                logger.exportCSV(path)
            with path.open(newline="") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual([r["rfid_available"] for r in rows], ["1", "1", "0", "1"])
            self.assertEqual([r["speed_at_limit"] for r in rows], ["1", "0", "1", "1"])
            self.assertEqual([r["turn_rate_at_limit"] for r in rows], ["0", "1", "0", "1"])

    def test_missing_uncertainty_reports_coverage_and_safe_json(self):
        logger = DataLogger()
        beforeAcquisition = entry(0)
        beforeAcquisition.estimatedTag = None
        beforeAcquisition.diagnostics = EstimatorDiagnostics()
        logger.log(beforeAcquisition)
        logger.log(entry(1, uncertainty=2))
        logger.log(entry(2, uncertainty=2))
        self.assertEqual(logger.stats()["uncertaintyTimeCoverage"], 50)
        self.assertEqual(logger.stats()["meanPositionUncertainty"], 2)
        logger.clear()
        logger.log(beforeAcquisition)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "summary.json"
            with redirect_stdout(io.StringIO()):
                logger.exportSummary(path)
            summary = json.loads(path.read_text())
            self.assertIsNone(summary["positionRMSE"])
            self.assertIsNone(summary["meanPositionUncertainty"])
            self.assertIsNone(summary["speedLimitUsage"])
            self.assertIsNone(summary["validTimeRate"])

    def test_maneuver_recovery_requires_continuous_hold(self):
        logger = DataLogger(recoveryThreshold=0.5, recoveryHoldTime=1)
        errors = [0.2, 2, 0.3, 0.6, 0.3, 0.3, 0.3]
        for i, error in enumerate(errors):
            logger.log(entry(i * 0.5, error, event="maneuver" if i == 1 else ""))
        result, = logger.recoveryStats()
        self.assertEqual(result["event"], "maneuver")
        self.assertEqual(result["status"], "recovered")
        self.assertEqual(result["recoveredAt"], 3)
        self.assertEqual(result["recoveryTime"], 2.5)

    def test_dropout_recovery_clock_starts_at_first_valid_read(self):
        logger = DataLogger(recoveryHoldTime=1)
        logger.log(entry(0, valid=False))
        logger.log(entry(0.5, due=False, valid=False))
        logger.log(entry(1, 0.9))
        logger.log(entry(1.5, 0.2, due=False, valid=False))
        logger.log(entry(2, 0.2))
        logger.log(entry(2.5, 0.2))
        result, = logger.recoveryStats()
        self.assertEqual(result["start"], 0)
        self.assertEqual(result["end"], 1)
        self.assertEqual(result["recoveryTime"], 1.5)

    def test_unrecovered_ongoing_and_interrupted_events(self):
        logger = DataLogger(recoveryHoldTime=1)
        logger.log(entry(0, 2, event="maneuver"))
        logger.log(entry(0.5, 2, event="disturbance"))
        logger.log(entry(1, 2))
        results = logger.recoveryStats()
        self.assertEqual([r["status"] for r in results], ["interrupted", "not_recovered"])
        self.assertTrue(all(r["recoveryTime"] is None for r in results))
        logger.clear()
        logger.log(entry(0, valid=False))
        logger.log(entry(1, valid=False))
        result, = logger.recoveryStats()
        self.assertEqual(result["status"], "ongoing")
        self.assertIsNone(result["end"])

    def test_cli_saves_diagnostics_and_event_recovery_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scenario.csv"
            path.write_text("time,x,y,vx,vy,event\n0,4,2,0,0,\n0.1,4,2,0,0,maneuver\n0.2,4,2,0,0,\n")
            output = Path(directory) / "results.csv"
            with redirect_stdout(io.StringIO()):
                main(["--scenario", str(path), "--output", str(output), "--dropout", "0",
                      "--recovery-threshold", "10", "--recovery-hold", "0", "--particles", "10"])
            summary = json.loads(output.with_suffix(".summary.json").read_text())
            result, = summary["recoveryEvents"]
            self.assertEqual(result["event"], "maneuver")
            self.assertEqual(result["start"], 0.1)
            self.assertEqual(result["recoveryTime"], 0)
            self.assertEqual(summary["maxVelocity"], 2)
            self.assertGreater(summary["meanPositionUncertainty"], 0)
            with output.open(newline="") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(rows[1]["event"], "maneuver")
            self.assertGreater(float(rows[0]["position_uncertainty"]), 0)


if __name__ == "__main__":
    unittest.main()
