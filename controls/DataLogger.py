"""Simulation snapshots, assignment metrics, and per-event recovery evaluation."""

import csv
import json
from copy import deepcopy
from dataclasses import dataclass, field
from math import hypot, isfinite, nan, sqrt
from pathlib import Path

from .Utils import ControlCommand, RFIDMeasurement, ReaderState, TagState, positive
from .EstimatorDiagnostics import EstimatorDiagnostics


@dataclass
class LogEntry:
    time: float
    trueTag: TagState
    estimatedTag: TagState | None
    reader: ReaderState
    measurement: RFIDMeasurement
    command: ControlCommand
    measurementDue: bool = True
    diagnostics: EstimatorDiagnostics = field(default_factory=EstimatorDiagnostics)
    event: str = ""


class DataLogger:
    def __init__(self, *, maxVelocity: float | None = None, maxTurnRate: float | None = None,
                 recoveryThreshold: float = 0.5, recoveryHoldTime: float = 1.0):
        for name, value in (("maxVelocity", maxVelocity), ("maxTurnRate", maxTurnRate)):
            if value is not None:
                positive(value, name, allowZero=True)
        positive(recoveryThreshold, "recoveryThreshold", allowZero=True)
        positive(recoveryHoldTime, "recoveryHoldTime", allowZero=True)
        self.maxVelocity = maxVelocity
        self.maxTurnRate = maxTurnRate
        self.recoveryThreshold = recoveryThreshold
        self.recoveryHoldTime = recoveryHoldTime
        self.entries: list[LogEntry] = []

    def log(self, entry: LogEntry) -> None:
        if not isfinite(entry.time) or (self.entries and entry.time <= self.entries[-1].time):
            raise ValueError("Log timestamps must be finite and strictly increasing")
        self.entries.append(deepcopy(entry))

    def _availability(self) -> list[bool]:
        available = False
        values = []
        for e in self.entries:
            if e.measurementDue:
                available = e.measurement.isValid
            values.append(available)
        return values

    @staticmethod
    def _atLimit(value: float, limit: float | None) -> bool | None:
        if limit is None or limit <= 0:
            return None
        return abs(value) >= limit - max(1e-9, limit * 1e-9)

    def recoveryStats(self) -> list[dict]:
        """Evaluate explicit CSV events and consecutive invalid-read episodes.

        CSV event timestamps start the recovery clock. A dropout's clock starts
        at the first subsequent valid read; an unfinished dropout has no end.
        """
        events = []
        dropoutStart = None
        for e in self.entries:
            if e.event:
                events.append({"event": e.event, "start": e.time, "end": e.time})
            if e.measurementDue:
                if not e.measurement.isValid and dropoutStart is None:
                    dropoutStart = e.time
                elif e.measurement.isValid and dropoutStart is not None:
                    events.append({"event": "dropout", "start": dropoutStart, "end": e.time})
                    dropoutStart = None
        if dropoutStart is not None:
            events.append({"event": "dropout", "start": dropoutStart, "end": None})
        events.sort(key=lambda event: event["start"])
        available = self._availability()
        for event in events:
            event.update(recoveryTime=None, recoveredAt=None, status="not_recovered")
            if event["end"] is None:
                event["status"] = "ongoing"
                continue
            # Do not claim recovery across a later disturbance.
            nextEvent = min((other["start"] for other in events
                             if other["start"] > event["start"]), default=float("inf"))
            streakStart = None
            for e, valid in zip(self.entries, available):
                if e.time < event["end"]:
                    continue
                if e.time >= nextEvent:
                    event["status"] = "interrupted"
                    break
                error = self._errors(e)[0]
                if valid and isfinite(error) and error <= self.recoveryThreshold:
                    if streakStart is None:
                        streakStart = e.time
                    if e.time - streakStart + 1e-12 >= self.recoveryHoldTime:
                        event.update(recoveryTime=e.time - event["end"], recoveredAt=e.time, status="recovered")
                        break
                else:
                    streakStart = None
        return events

    @staticmethod
    def _errors(entry: LogEntry) -> tuple[float, float]:
        true = entry.trueTag.position
        estimate = entry.estimatedTag
        error = hypot(true.x - estimate.position.x, true.y - estimate.position.y) if estimate else nan
        distance = hypot(true.x - entry.reader.position.x, true.y - entry.reader.position.y)
        return error, distance

    def printLatest(self) -> None:
        if not self.entries:
            return
        e = self.entries[-1]
        error, distance = self._errors(e)
        print(f"\nTime: {e.time:.3f} s")
        print(f"True tag: ({e.trueTag.position.x:.3f}, {e.trueTag.position.y:.3f})")
        if e.estimatedTag is None:
            print("Estimated tag: unavailable (waiting for acquisition)")
        else:
            print(f"Estimated tag: ({e.estimatedTag.position.x:.3f}, {e.estimatedTag.position.y:.3f})")
        print(f"Reader: ({e.reader.position.x:.3f}, {e.reader.position.y:.3f})")
        print(f"Estimation error: {error:.3f} m")
        print(f"Tracking distance: {distance:.3f} m")
        print(f"RFID valid: {str(e.measurement.isValid).lower()}")

    def stats(self) -> dict:
        errors = [self._errors(e)[0] for e in self.entries if e.estimatedTag is not None]
        attempts = sum(e.measurementDue for e in self.entries)
        intervals = [b.time - a.time for a, b in zip(self.entries, self.entries[1:])] + ([0.0] if self.entries else [])
        duration = sum(intervals)
        uncertainty = [e.diagnostics.positionUncertainty for e in self.entries]
        knownUncertaintyTime = sum(dt for dt, u in zip(intervals, uncertainty) if isfinite(u))
        def limitUsage(attribute, limit):
            if not duration or limit is None or limit <= 0:
                return nan
            return 100 * sum(dt for e, dt in zip(self.entries, intervals)
                             if self._atLimit(getattr(e.command, attribute), limit)) / duration
        result = {
            "samples": len(self.entries),
            "duration": self.entries[-1].time - self.entries[0].time if self.entries else 0.0,
            "positionRMSE": sqrt(sum(x * x for x in errors) / len(errors)) if errors else nan,
            "maxError": max(errors) if errors else nan,
            "meanTrackingDistance": sum(self._errors(e)[1] for e in self.entries) / len(self.entries) if self.entries else nan,
            "measurementAttempts": attempts,
            "validRate": 100.0 * sum(e.measurement.isValid for e in self.entries) / attempts if attempts else nan,
            "estimatedSamples": len(errors),
            "validTimeRate": 100 * sum(dt for dt, valid in zip(intervals, self._availability()) if valid) / duration if duration else nan,
            "meanPositionUncertainty": sum(dt * u for dt, u in zip(intervals, uncertainty) if isfinite(u)) / knownUncertaintyTime if knownUncertaintyTime else nan,
            "maxPositionUncertainty": max((u for u in uncertainty if isfinite(u)), default=nan),
            "uncertaintyTimeCoverage": 100 * knownUncertaintyTime / duration if duration else nan,
            "maxTrackingDistance": max((self._errors(e)[1] for e in self.entries), default=nan),
            "speedLimitUsage": limitUsage("velocity", self.maxVelocity),
            "turnRateLimitUsage": limitUsage("turnRate", self.maxTurnRate),
            "maxVelocity": self.maxVelocity,
            "maxTurnRate": self.maxTurnRate,
            "recoveryThreshold": self.recoveryThreshold,
            "recoveryHoldTime": self.recoveryHoldTime,
        }
        result["recoveryEvents"] = self.recoveryStats()
        return result

    def printStats(self) -> None:
        if not self.entries:
            print("No simulation data.")
            return
        s = self.stats()
        print("\n========== SIMULATION STATS ==========")
        print(f"Samples: {s['samples']}")
        print(f"Duration: {s['duration']:.3f} s")
        print(f"Position RMSE: {s['positionRMSE']:.3f} m")
        print(f"Max estimation error: {s['maxError']:.3f} m")
        print(f"Mean tracking distance: {s['meanTrackingDistance']:.3f} m")
        print(f"RFID valid rate: {s['validRate']:.3f}% ({s['measurementAttempts']} attempts)")
        print(f"Estimated samples: {s['estimatedSamples']}/{s['samples']}")
        print(f"Valid-read time availability: {s['validTimeRate']:.3f}%")
        print(f"Position uncertainty (mean/max): {s['meanPositionUncertainty']:.3f} / {s['maxPositionUncertainty']:.3f} m")
        print(f"Uncertainty time coverage: {s['uncertaintyTimeCoverage']:.3f}%")
        print(f"Max tracking distance: {s['maxTrackingDistance']:.3f} m")
        print(f"Time at command limits (speed/turn): {s['speedLimitUsage']:.3f}% / {s['turnRateLimitUsage']:.3f}%")
        print(f"Recovery rule: error <= {self.recoveryThreshold:g} m for {self.recoveryHoldTime:g} s with valid RFID")
        for event in s["recoveryEvents"]:
            recovery = f"{event['recoveryTime']:.3f} s" if event['recoveryTime'] is not None else event['status']
            print(f"Recovery [{event['event']} at {event['start']:.3f} s]: {recovery}")
        print("======================================")

    def exportCSV(self, filename: str | Path) -> None:
        with open(filename, "w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow([
                "time", "true_x", "true_y", "true_vx", "true_vy",
                "est_x", "est_y", "est_vx", "est_vy",
                "reader_x", "reader_y", "heading", "velocity", "turnSpeed",
                "rssi", "valid", "uncertainty", "bearing",
                "command_velocity", "command_turnRate", "estimation_error", "tracking_distance",
                "measurement_due",
                "position_cov_xx", "position_cov_xy", "position_cov_yy", "position_uncertainty",
                "effective_sample_size", "resampled", "rfid_available",
                "speed_at_limit", "turn_rate_at_limit", "event",
            ])
            for e, available in zip(self.entries, self._availability()):
                estimate = e.estimatedTag
                speedAtLimit = self._atLimit(e.command.velocity, self.maxVelocity)
                turnAtLimit = self._atLimit(e.command.turnRate, self.maxTurnRate)
                est = [estimate.position.x, estimate.position.y, estimate.xdot, estimate.ydot] if estimate else [nan] * 4
                writer.writerow([
                    e.time, e.trueTag.position.x, e.trueTag.position.y, e.trueTag.xdot, e.trueTag.ydot,
                    *est, e.reader.position.x, e.reader.position.y, e.reader.heading,
                    e.reader.velocity, e.reader.turnSpeed, e.measurement.Pr, int(e.measurement.isValid),
                    e.measurement.uncertainty, e.measurement.bearing, e.command.velocity, e.command.turnRate,
                    *self._errors(e), int(e.measurementDue),
                    e.diagnostics.covarianceXX, e.diagnostics.covarianceXY, e.diagnostics.covarianceYY,
                    e.diagnostics.positionUncertainty, e.diagnostics.effectiveSampleSize,
                    int(e.diagnostics.resampled), int(available),
                    int(speedAtLimit) if speedAtLimit is not None else "",
                    int(turnAtLimit) if turnAtLimit is not None else "", e.event,
                ])
        print(f"Exported {len(self.entries)} samples to {filename}")

    def exportSummary(self, filename: str | Path) -> None:
        def serializable(value):
            if isinstance(value, dict):
                return {key: serializable(item) for key, item in value.items()}
            if isinstance(value, list):
                return [serializable(item) for item in value]
            return None if isinstance(value, float) and not isfinite(value) else value
        with open(filename, "w", encoding="utf-8") as file:
            json.dump(serializable(self.stats()), file, indent=2, allow_nan=False)
            file.write("\n")
        print(f"Exported summary metrics to {filename}")

    def clear(self) -> None:
        self.entries.clear()
