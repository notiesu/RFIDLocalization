"""Particle-cloud snapshots, captured before resampling at each estimator tick."""

import csv
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from math import nan

from .EstimatorDiagnostics import EstimatorDiagnostics
from .Utils import Particle, ReaderState, RFIDMeasurement, TagState


@dataclass(frozen=True, slots=True)
class ParticleSnapshot:
    x: float
    y: float
    xdot: float
    ydot: float
    weight: float


@dataclass(frozen=True)
class EstimatorLogEntry:
    time: float
    particles: tuple[ParticleSnapshot, ...]
    estimatedTag: TagState | None
    reader: ReaderState
    measurement: RFIDMeasurement | None
    effectiveSampleSize: float
    resampled: bool  # Whether resampling follows this snapshot
    diagnostics: EstimatorDiagnostics


class EstimatorLogger:
    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.entries: list[EstimatorLogEntry] = []
        self.latestDiagnostics = EstimatorDiagnostics()

    def log(
        self, time: float, particles: list[Particle], estimate: TagState | None,
        reader: ReaderState, measurement: RFIDMeasurement | None,
        effectiveSampleSize: float, resampled: bool,
    ) -> None:
        covariance = (nan, nan, nan)
        if estimate is not None and particles:
            dx = [p.state.position.x - estimate.position.x for p in particles]
            dy = [p.state.position.y - estimate.position.y for p in particles]
            covariance = (
                sum(p.weight * x * x for p, x in zip(particles, dx)),
                sum(p.weight * x * y for p, x, y in zip(particles, dx, dy)),
                sum(p.weight * y * y for p, y in zip(particles, dy)),
            )
        self.latestDiagnostics = EstimatorDiagnostics(*covariance, effectiveSampleSize, resampled)
        if not self.enabled:
            return
        snapshots = tuple(ParticleSnapshot(p.state.position.x, p.state.position.y,
                                           p.state.xdot, p.state.ydot, p.weight)
                          for p in particles)
        self.entries.append(EstimatorLogEntry(
            time, snapshots, deepcopy(estimate), deepcopy(reader), deepcopy(measurement),
            effectiveSampleSize, resampled, self.latestDiagnostics,
        ))

    def exportCSV(self, filename: str | Path) -> None:
        """One row per particle per tick; empty clouds get one blank-particle row.

        particle_index is a position within that tick's cloud, not an identity
        maintained across resampling.
        """
        with open(filename, "w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow([
                "time", "particle_index", "x", "y", "vx", "vy", "weight",
                "effective_sample_size", "resampled", "measurement_due", "valid",
                "reader_x", "reader_y", "heading", "est_x", "est_y", "est_vx", "est_vy",
                "position_cov_xx", "position_cov_xy", "position_cov_yy", "position_uncertainty",
            ])
            for e in self.entries:
                estimate = e.estimatedTag
                metadata = [
                    e.effectiveSampleSize, int(e.resampled), int(e.measurement is not None),
                    int(e.measurement.isValid) if e.measurement is not None else 0,
                    e.reader.position.x, e.reader.position.y, e.reader.heading,
                    *([estimate.position.x, estimate.position.y, estimate.xdot, estimate.ydot]
                      if estimate is not None else [""] * 4),
                    e.diagnostics.covarianceXX, e.diagnostics.covarianceXY,
                    e.diagnostics.covarianceYY, e.diagnostics.positionUncertainty,
                ]
                if not e.particles:
                    writer.writerow([e.time, *[""] * 6, *metadata])
                for index, p in enumerate(e.particles):
                    writer.writerow([e.time, index, p.x, p.y, p.xdot, p.ydot, p.weight, *metadata])
        print(f"Exported {len(self.entries)} particle-cloud snapshots to {filename}")

    def clear(self) -> None:
        self.entries.clear()
        self.latestDiagnostics = EstimatorDiagnostics()
