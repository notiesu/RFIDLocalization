"""Run RFID tracking against an externally generated ground-truth scenario CSV."""

import argparse
from pathlib import Path

from .Controller import Controller, ControllerConfig
from .Estimator import Estimator
from .ReaderPlant import ReaderPlant
from .RFIDSensor import RFIDConfig, RFIDSensor
from .ScenarioGenerator import ScenarioGenerator, loadScenarioCSV
from .Simulation import Simulation
from .Utils import ReaderState


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True,
                        help="ground-truth CSV with columns time,x,y,vx,vy; uniform timestamps starting at 0")
    parser.add_argument("--update-rate", type=float, default=10.0, help="sensor sampling rate in Hz")
    parser.add_argument("--dropout", type=float, default=0.05, help="probability per sensor attempt")
    parser.add_argument("--particles", type=int, default=500)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=Path, default=Path("simulation.csv"))
    parser.add_argument("--plot", type=Path, help="optional figure file (requires matplotlib)")
    parser.add_argument("--particle-log", type=Path, help="export particle snapshots as CSV")
    parser.add_argument("--particle-animation", type=Path,
                        help="export an interactive particle-cloud animation (.html)")
    args = parser.parse_args(argv)
    if args.particle_animation and args.particle_animation.suffix.lower() != ".html":
        parser.error("--particle-animation must end in .html")
    for output in (args.output, args.plot, args.particle_log, args.particle_animation):
        if output is not None and output.resolve() == args.scenario.resolve():
            parser.error("Output files must differ from the input scenario CSV")
    try:
        config = RFIDConfig(updateRate=args.update_rate, dropoutProbability=args.dropout)
        scenario = loadScenarioCSV(args.scenario)
        simulation = Simulation(
            ScenarioGenerator(scenario), RFIDSensor(config, args.seed),
            Estimator(config, particleCount=args.particles, seed=args.seed + 1),
            Controller(ControllerConfig()), ReaderPlant(ReaderState()),
        )
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if args.plot or args.particle_animation:
        try:
            from .Visualization import animateParticles, plotSimulation
            import matplotlib  # noqa: F401
        except ImportError:
            parser.error("Visualization requires matplotlib; install with: python -m pip install -r requirements.txt")
    logger = simulation.run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    logger.exportCSV(args.output)
    logger.printStats()
    if args.particle_log:
        args.particle_log.parent.mkdir(parents=True, exist_ok=True)
        simulation.estimator.logger.exportCSV(args.particle_log)
    if args.plot:
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        plotSimulation(logger, args.plot)
        print(f"Saved figure to {args.plot}")
    if args.particle_animation:
        args.particle_animation.parent.mkdir(parents=True, exist_ok=True)
        animateParticles(simulation.estimator.logger, args.particle_animation, logger)
        print(f"Saved particle animation to {args.particle_animation}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
