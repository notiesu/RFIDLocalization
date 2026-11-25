"""Run RFID tracking against an externally generated ground-truth scenario CSV."""

import argparse
from pathlib import Path

from .Controller import Controller, ControllerConfig
from .DataLogger import DataLogger
from .EKFestimator import EKFEstimator
from .ParticleFilterEstimator import ParticleFilterEstimator
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
    parser.add_argument("--estimator", choices=("particle", "ekf"), default="particle")
    parser.add_argument("--ekf-gate", choices=("monitor", "reject", "off"), default="monitor",
                        help="normalized-innovation policy (EKF only; default: monitor)")
    parser.add_argument("--ekf-gate-threshold", type=float, default=EKFEstimator.DEFAULT_INNOVATION_THRESHOLD,
                        help="NIS threshold; default 9.21034 is nominal 99%% for 2 measurements")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--recovery-threshold", type=float, default=0.5,
                        help="recovered position-error threshold in meters (default: 0.5)")
    parser.add_argument("--recovery-hold", type=float, default=1.0,
                        help="required continuous recovery hold in seconds (default: 1)")
    parser.add_argument("--output", type=Path, default=Path("simulation.csv"))
    parser.add_argument("--plot", type=Path, help="optional figure file (requires matplotlib)")
    parser.add_argument("--animation", type=Path, help="offline motion HTML for either estimator (.html)")
    parser.add_argument("--particle-log", type=Path, help="export particle snapshots as CSV")
    parser.add_argument("--particle-animation", type=Path,
                        help="export an interactive particle-cloud animation (.html)")
    args = parser.parse_args(argv)
    if args.estimator == "ekf" and (args.particle_log or args.particle_animation):
        parser.error("Particle logs and particle animations require --estimator particle; use --animation or --plot for EKF results")
    if args.particle_animation and args.particle_animation.suffix.lower() != ".html":
        parser.error("--particle-animation must end in .html")
    if args.animation and args.animation.suffix.lower() != ".html":
        parser.error("--animation must end in .html")
    summaryPath = args.output.with_suffix(".summary.json")
    for output in (args.output, summaryPath, args.plot, args.animation, args.particle_log, args.particle_animation):
        if output is not None and output.resolve() == args.scenario.resolve():
            parser.error("Output files must differ from the input scenario CSV")
    try:
        config = RFIDConfig(updateRate=args.update_rate, dropoutProbability=args.dropout)
        scenario = loadScenarioCSV(args.scenario)
        estimator = (EKFEstimator(config, innovationGateThreshold=args.ekf_gate_threshold,
                                  innovationGateMode=args.ekf_gate) if args.estimator == "ekf" else
                     ParticleFilterEstimator(config, particleCount=args.particles, seed=args.seed + 1))
        simulation = Simulation(
            ScenarioGenerator(scenario), RFIDSensor(config, args.seed),
            estimator,
            Controller(ControllerConfig()), ReaderPlant(ReaderState()),
            DataLogger(recoveryThreshold=args.recovery_threshold, recoveryHoldTime=args.recovery_hold),
        )
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if args.plot or args.particle_animation or args.animation:
        try:
            from .Visualization import animateParticles, animateSimulation, plotSimulation
            import matplotlib  # noqa: F401
        except ImportError:
            parser.error("Visualization requires matplotlib; install with: python -m pip install -r requirements.txt")
    logger = simulation.run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    logger.exportCSV(args.output)
    logger.exportSummary(summaryPath)
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
    if args.animation:
        args.animation.parent.mkdir(parents=True, exist_ok=True)
        if args.estimator == "particle":
            animateParticles(simulation.estimator.logger, args.animation, logger)
        else:
            animateSimulation(logger, args.animation, title="EKF tracking")
        print(f"Saved motion animation to {args.animation}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
