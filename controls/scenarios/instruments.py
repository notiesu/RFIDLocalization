"""Generate external randomized instruments, one row per outgoing interval."""

import argparse
import csv
from dataclasses import dataclass
from math import isclose, isfinite
from pathlib import Path
from random import Random


@dataclass(frozen=True)
class InstrumentSample:
    time: float
    z_x: float
    z_y: float


def instrumentTimestep(samples: list[InstrumentSample]) -> float:
    if len(samples) < 2:
        raise ValueError("Need at least two instrument rows to determine dt")
    if not all(isfinite(v) for s in samples for v in (s.time, s.z_x, s.z_y)):
        raise ValueError("Instrument values must be finite")
    dt = samples[1].time - samples[0].time
    if samples[0].time != 0 or dt <= 0 or any(
        s.time <= samples[i - 1].time or
        not isclose(s.time, i * dt, rel_tol=1e-7, abs_tol=1e-9)
        for i, s in enumerate(samples[1:], 1)
    ):
        raise ValueError("Instrument times must start at zero and have uniform positive spacing")
    return dt


def generateInstruments(duration: float = 20, dt: float = 0.1,
                        stddev: float = 1, seed: int = 42) -> list[InstrumentSample]:
    """Independent zero-mean Gaussian instruments in each direction.

    A separate RNG namespace keeps these draws separate from trajectory noise,
    even when both modules receive the same numeric seed.
    """
    if not all(isfinite(v) for v in (duration, dt, stddev)) or dt <= 0 or stddev <= 0:
        raise ValueError("Require finite duration, dt > 0, and stddev > 0")
    count = round(duration / dt)
    if count < 2 or not isclose(count * dt, duration, rel_tol=1e-9, abs_tol=1e-9):
        raise ValueError("Duration must be an integer multiple of dt, with at least two intervals")
    rng = Random(f"instrument:{seed}")
    return [InstrumentSample(i * dt, rng.gauss(0, stddev), rng.gauss(0, stddev))
            for i in range(count)]


def writeInstrumentCSV(filename: str | Path, samples: list[InstrumentSample]) -> None:
    instrumentTimestep(samples)
    path = Path(filename)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("time", "z_x", "z_y"))
        writer.writerows((s.time, s.z_x, s.z_y) for s in samples)


def loadInstrumentCSV(filename: str | Path) -> list[InstrumentSample]:
    try:
        with open(filename, newline="", encoding="utf-8-sig") as file:
            reader = csv.DictReader(file, strict=True)
            headers = reader.fieldnames or []
            if not {"time", "z_x", "z_y"}.issubset(headers) or len(headers) != len(set(headers)):
                raise ValueError("Instrument CSV needs unique headers including time,z_x,z_y")
            samples = []
            for row in reader:
                if None in row:
                    raise ValueError("Too many instrument CSV fields")
                samples.append(InstrumentSample(*(float(row[k]) for k in ("time", "z_x", "z_y"))))
    except (csv.Error, TypeError, KeyError) as exc:
        raise ValueError(f"Invalid instrument CSV: {exc}") from exc
    instrumentTimestep(samples)
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=20)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--stddev", type=float, default=1)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    try:
        samples = generateInstruments(args.duration, args.dt, args.stddev, args.seed)
        writeInstrumentCSV(args.output, samples)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f"Wrote {len(samples)} instrument intervals to {args.output}")


if __name__ == "__main__":
    main()
