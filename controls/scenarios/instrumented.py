"""Generate an endogenous-input trajectory from externally supplied instruments."""

import argparse
import csv
import json
from copy import deepcopy
from dataclasses import asdict, dataclass
from math import isfinite, sqrt
from pathlib import Path
from random import Random

from ..Utils import Coord, Scenario, TagState
from .instruments import InstrumentSample, instrumentTimestep, loadInstrumentCSV


@dataclass(frozen=True)
class MixtureDistribution:
    probability: float
    means: tuple[float, float]
    stddevs: tuple[float, float]

    @classmethod
    def draw(cls, rng: Random):
        """Draw one non-Gaussian law, normalized to mean zero and variance one."""
        p = rng.uniform(0.25, 0.75)
        m = rng.uniform(0.8, 1.5)
        means = (m, -p * m / (1 - p))
        sigmas = (rng.uniform(0.15, 0.45), rng.uniform(0.15, 0.45))
        scale = sqrt(p * (sigmas[0] ** 2 + means[0] ** 2)
                     + (1 - p) * (sigmas[1] ** 2 + means[1] ** 2))
        return cls(p, tuple(v / scale for v in means), tuple(v / scale for v in sigmas))

    def sample(self, rng: Random) -> float:
        component = 0 if rng.random() < self.probability else 1
        return rng.gauss(self.means[component], self.stddevs[component])


@dataclass
class GeneratedTrajectory:
    scenario: Scenario
    instruments: list[InstrumentSample]
    inputs: list[tuple[float, float]]
    truth: dict


def generateTrajectory(
    instruments: list[InstrumentSample], *, alpha: float = 1, beta: float = 0.5,
    gamma: float = 0.3, delta: float = 0.8, inputNoiseStddev: float = 0.2,
    seed: int = 42, distributionSeed: int | None = None,
    initialState: TagState | None = None,
) -> GeneratedTrajectory:
    """Use the supplied Euler equations; no clipping or extra position noise.

    The hidden mixture law is fixed for the whole trajectory. Its draws are iid
    across axes/time; nu is independent Gaussian input noise. Supplied z must be
    generated independently of both disturbances for the intended IV assumptions.
    """
    dt = instrumentTimestep(instruments)
    if not all(isfinite(v) for v in (alpha, beta, gamma, delta, inputNoiseStddev)) or inputNoiseStddev < 0:
        raise ValueError("Coefficients must be finite and inputNoiseStddev nonnegative")
    initial = deepcopy(initialState) if initialState is not None else TagState(Coord(2, 2))
    if not all(isfinite(v) for v in (initial.position.x, initial.position.y, initial.xdot, initial.ydot)):
        raise ValueError("Initial state must be finite")
    lawSeed = seed if distributionSeed is None else distributionSeed
    law = MixtureDistribution.draw(Random(f"distribution:{lawSeed}"))
    wRng, nuRng = Random(f"disturbance:{seed}"), Random(f"input-noise:{seed}")
    states, inputs, latent = [initial], [], []
    for z in instruments:
        w = (law.sample(wRng), law.sample(wRng))
        nu = (nuRng.gauss(0, inputNoiseStddev), nuRng.gauss(0, inputNoiseStddev))
        u = tuple(alpha * zi + delta * wi + ni for zi, wi, ni in zip((z.z_x, z.z_y), w, nu))
        s = states[-1]
        nextState = TagState(
            Coord(s.position.x + s.xdot * dt, s.position.y + s.ydot * dt),
            s.xdot + (beta * u[0] + gamma * w[0]) * dt,
            s.ydot + (beta * u[1] + gamma * w[1]) * dt,
        )
        if not all(isfinite(v) for v in (*u, nextState.position.x, nextState.position.y,
                                         nextState.xdot, nextState.ydot)):
            raise ValueError("Generated trajectory overflowed; reduce coefficients or duration")
        states.append(nextState)
        inputs.append(u)
        latent.append({"time": z.time, "w": w, "nu": nu})
    truth = {
        "schemaVersion": 1, "seed": seed, "distributionSeed": lawSeed,
        "coefficients": {"alpha": alpha, "beta": beta, "gamma": gamma, "delta": delta},
        "disturbance": {"family": "two_component_gaussian_mixture", **asdict(law),
                        "mean": 0, "variance": 1, "iidAcrossTimeAndAxes": True},
        "inputNoise": {"family": "gaussian", "mean": 0, "stddev": inputNoiseStddev},
        "dt": dt, "discretization": "forward Euler: position uses previous velocity",
        "latentDraws": latent,
    }
    return GeneratedTrajectory(Scenario(dt, states), list(instruments), inputs, truth)


def writeTrajectoryCSV(filename: str | Path, result: GeneratedTrajectory) -> tuple[Path, Path]:
    """Write observed states/inputs and a separate evaluation-only truth sidecar."""
    path = Path(filename)
    truthPath = path.with_suffix(".truth.json")
    if path.resolve() == truthPath.resolve():
        raise ValueError("Trajectory filename must differ from its .truth.json sidecar")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("time", "x", "y", "vx", "vy", "u_x", "u_y", "z_x", "z_y"))
        for i, s in enumerate(result.scenario.states):
            outgoing = (*result.inputs[i], result.instruments[i].z_x, result.instruments[i].z_y) \
                if i < len(result.inputs) else ("", "", "", "")
            writer.writerow((i * result.scenario.dt, s.position.x, s.position.y,
                             s.xdot, s.ydot, *outgoing))
    truthPath.write_text(json.dumps(result.truth, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return path, truthPath


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instruments", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--distribution-seed", type=int)
    for name, default in (("alpha", 1), ("beta", 0.5), ("gamma", 0.3), ("delta", 0.8)):
        parser.add_argument(f"--{name}", type=float, default=default)
    parser.add_argument("--input-noise-stddev", type=float, default=0.2)
    args = parser.parse_args()
    try:
        if args.instruments.resolve() in (args.output.resolve(), args.output.with_suffix(".truth.json").resolve()):
            raise ValueError("Outputs must not overwrite the instrument input")
        result = generateTrajectory(loadInstrumentCSV(args.instruments), alpha=args.alpha,
                                    beta=args.beta, gamma=args.gamma, delta=args.delta,
                                    inputNoiseStddev=args.input_noise_stddev, seed=args.seed,
                                    distributionSeed=args.distribution_seed)
        paths = writeTrajectoryCSV(args.output, result)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f"Wrote trajectory to {paths[0]} and evaluation truth to {paths[1]}")


if __name__ == "__main__":
    main()
