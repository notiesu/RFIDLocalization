"""Value-snapshot logging with the original C++ CSV columns and statistics."""

import csv
from copy import deepcopy
from dataclasses import dataclass
from math import hypot, nan, sqrt
from pathlib import Path

from .Utils import ControlCommand, RFIDMeasurement, ReaderState, TagState


@dataclass
class LogEntry:
    time: float
    trueTag: TagState
    estimatedTag: TagState | None
    reader: ReaderState
    measurement: RFIDMeasurement
    command: ControlCommand
    measurementDue: bool = True


class DataLogger:
    def __init__(self):
        self.entries: list[LogEntry] = []

    def log(self, entry: LogEntry) -> None:
        self.entries.append(deepcopy(entry))

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

    def stats(self) -> dict[str, float | int]:
        errors = [self._errors(e)[0] for e in self.entries if e.estimatedTag is not None]
        attempts = sum(e.measurementDue for e in self.entries)
        return {
            "samples": len(self.entries),
            "duration": self.entries[-1].time - self.entries[0].time if self.entries else 0.0,
            "positionRMSE": sqrt(sum(x * x for x in errors) / len(errors)) if errors else nan,
            "maxError": max(errors) if errors else nan,
            "meanTrackingDistance": sum(self._errors(e)[1] for e in self.entries) / len(self.entries) if self.entries else nan,
            "measurementAttempts": attempts,
            "validRate": 100.0 * sum(e.measurement.isValid for e in self.entries) / attempts if attempts else nan,
            "estimatedSamples": len(errors),
        }

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
            ])
            for e in self.entries:
                estimate = e.estimatedTag
                est = [estimate.position.x, estimate.position.y, estimate.xdot, estimate.ydot] if estimate else [nan] * 4
                writer.writerow([
                    e.time, e.trueTag.position.x, e.trueTag.position.y, e.trueTag.xdot, e.trueTag.ydot,
                    *est, e.reader.position.x, e.reader.position.y, e.reader.heading,
                    e.reader.velocity, e.reader.turnSpeed, e.measurement.Pr, int(e.measurement.isValid),
                    e.measurement.uncertainty, e.measurement.bearing, e.command.velocity, e.command.turnRate,
                    *self._errors(e), int(e.measurementDue),
                ])
        print(f"Exported {len(self.entries)} samples to {filename}")

    def clear(self) -> None:
        self.entries.clear()
