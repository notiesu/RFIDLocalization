"""Generate the three trajectory categories required on assignment page 4."""

import argparse
import csv
from math import cos, pi, sin
from pathlib import Path


DEFAULT_OUTPUT = Path(__file__).resolve().parents[2] / "scenarios" / "assignment"


def generateScenarios(outputDir: Path) -> list[Path]:
    """Use a [0,10] x [0,10] m workspace; reader initially at (0,0).

    All tags start at (2,2), travel at 0.3 m/s, and run for 20 s at 10 Hz.
    Exact paths and numeric parameters are our choices, not prescribed values.
    """
    duration, dt, speed = 20.0, 0.1, 0.3
    radius = speed * duration / (pi / 2)
    rows = {"straight": [], "turning": [], "abrupt_maneuver": []}
    for i in range(round(duration / dt) + 1):
        time = i * dt
        rows["straight"].append((time, 2 + speed * time, 2, speed, 0))
        angle = speed * time / radius
        rows["turning"].append((
            time, 2 + radius * sin(angle), 2 + radius * (1 - cos(angle)),
            speed * cos(angle), speed * sin(angle),
        ))
        # Position stays continuous, but velocity jumps east -> north at t=10 s.
        if time < 10:
            maneuver = (time, 2 + speed * time, 2, speed, 0)
        else:
            maneuver = (time, 5, 2 + speed * (time - 10), 0, speed)
        rows["abrupt_maneuver"].append(maneuver)
    outputDir = Path(outputDir)
    outputDir.mkdir(parents=True, exist_ok=True)
    paths = []
    for name, samples in rows.items():
        path = outputDir / f"{name}.csv"
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(("time", "x", "y", "vx", "vy", "event"))
            for sample in samples:
                event = "abrupt_maneuver" if name == "abrupt_maneuver" and sample[0] == 10 else ""
                writer.writerow((*sample, event))
        paths.append(path)
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        paths = generateScenarios(args.output_dir)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f"Generated {len(paths)} assignment scenarios in {args.output_dir}")


if __name__ == "__main__":
    main()
