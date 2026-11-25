"""Generate Gaussian and variance-matched, heavy-tailed Student-t random walks."""

import argparse
import csv
from math import floor, isfinite, sqrt
from pathlib import Path
from random import Random


DEFAULT_OUTPUT = Path(__file__).resolve().parents[2] / "scenarios" / "randomwalk"


def generateScenarios(
    outputDir: Path, duration: float = 20.0, dt: float = 0.1,
    diffusion: float = 0.3, seed: int = 42,
) -> list[Path]:
    """Per-axis increment variance is diffusion² * dt for both walks.

    Start at (5,5). Positions are not clipped or reflected, so sampled increment
    distributions are preserved. Velocities describe each outgoing interval;
    the final row repeats the last interval's velocity.
    """
    if not all(isfinite(v) for v in (duration, dt, diffusion)) or dt <= 0 or duration < dt or diffusion < 0:
        raise ValueError("Require finite duration >= dt > 0 and diffusion >= 0")
    steps = floor(duration / dt + 1e-10)
    outputDir = Path(outputDir)
    outputDir.mkdir(parents=True, exist_ok=True)
    paths = []
    for distribution in ("gaussian", "student_t"):
        rng = Random(seed)
        positions = [(5.0, 5.0)]
        velocities = []

        def innovation():
            normal = rng.gauss(0, 1)
            if distribution == "gaussian":
                return normal
            # Student-t with df=3 has variance 3; standardize to unit variance.
            chiSquared = rng.gammavariate(1.5, 2)
            return normal / sqrt(chiSquared / 3) / sqrt(3)

        for _ in range(steps):
            dx, dy = (diffusion * sqrt(dt) * innovation() for _ in range(2))
            x, y = positions[-1]
            positions.append((x + dx, y + dy))
            velocities.append((dx / dt, dy / dt))
        velocities.append(velocities[-1])
        path = outputDir / f"{distribution}.csv"
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(("time", "x", "y", "vx", "vy"))
            for i, ((x, y), (vx, vy)) in enumerate(zip(positions, velocities)):
                writer.writerow((i * dt, x, y, vx, vy))
        paths.append(path)
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--duration", type=float, default=20)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--diffusion", type=float, default=0.3, help="per-axis scale in m/sqrt(s)")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    try:
        paths = generateScenarios(args.output_dir, args.duration, args.dt, args.diffusion, args.seed)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f"Generated {len(paths)} random-walk scenarios in {args.output_dir}")


if __name__ == "__main__":
    main()
