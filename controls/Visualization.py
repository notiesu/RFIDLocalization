"""Static plots and offline HTML animations for estimator runs."""

from pathlib import Path

from .DataLogger import DataLogger
from .EstimatorLogger import EstimatorLogger


def plotSimulation(logger: DataLogger, filename: str | Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    if not logger.entries:
        raise ValueError("No simulation data to plot")
    entries = logger.entries
    fig, (trajectory, errors) = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
    trajectory.plot([e.trueTag.position.x for e in entries],
                    [e.trueTag.position.y for e in entries], label="True tag")
    trajectory.plot([e.estimatedTag.position.x if e.estimatedTag else float("nan") for e in entries],
                    [e.estimatedTag.position.y if e.estimatedTag else float("nan") for e in entries],
                    label="Estimated tag", alpha=0.8)
    trajectory.plot([e.reader.position.x for e in entries],
                    [e.reader.position.y for e in entries], label="Reader")
    trajectory.set(title="RFID tracking", xlabel="x (m)", ylabel="y (m)", aspect="equal")
    times = [e.time for e in entries]
    errors.plot(times, [logger._errors(e)[0] for e in entries], label="Estimation error")
    errors.plot(times, [logger._errors(e)[1] for e in entries], label="Tracking distance")
    errors.set(title="Tracking performance", xlabel="Time (s)", ylabel="Distance (m)")
    for axis in (trajectory, errors):
        axis.legend()
        axis.grid(alpha=0.25)
    fig.savefig(filename, dpi=160)
    plt.close(fig)


def animateParticles(
    logger: EstimatorLogger, filename: str | Path,
    simulationLogger: DataLogger | None = None,
) -> None:
    """Export all logged clouds as an offline HTML animation with frame controls.

    Ground truth is an optional visualization overlay from the simulation logger;
    it is never passed into the estimator or its particle logger.
    """
    from math import isclose

    import matplotlib

    matplotlib.use("Agg")
    import numpy as np
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.colors import Normalize

    if Path(filename).suffix.lower() != ".html":
        raise ValueError("Particle animation filename must end in .html")
    if not logger.entries:
        raise ValueError("No particle snapshots to animate")
    entries = logger.entries
    reference = simulationLogger.entries if simulationLogger is not None else None
    if reference is not None and (
        len(reference) != len(entries)
        or any(not isclose(a.time, b.time, abs_tol=1e-9) for a, b in zip(entries, reference))
    ):
        raise ValueError("Particle and simulation logs must have matching timestamps")

    fig, axis = plt.subplots(figsize=(8, 6), dpi=90)
    try:
        fig.subplots_adjust(bottom=0.20, top=0.86)
        cloud = axis.scatter([], [], c=[], s=14, alpha=0.8, cmap="viridis",
                             norm=Normalize(0, 5), label="Particles")
        fig.colorbar(cloud, ax=axis, label="Weight / uniform weight", extend="max")
        readerTrail, = axis.plot([], [], color="tab:blue", alpha=0.25)
        estimateTrail, = axis.plot([], [], color="tab:orange", alpha=0.4)
        readerMarker, = axis.plot([], [], "s", color="tab:blue", label="Reader")
        estimateMarker, = axis.plot([], [], "x", color="tab:orange", markersize=9, label="Estimate")
        trueTrail, = axis.plot([], [], color="tab:red", alpha=0.3)
        trueMarker, = axis.plot([], [], "*", color="tab:red", markersize=11,
                                label="True tag" if reference is not None else "_nolegend_")
        title = fig.suptitle("Particle cloud")
        status = fig.text(0.10, 0.07, "", fontsize=10)
        axis.set(xlabel="x (m)", ylabel="y (m)", aspect="equal")
        axis.grid(alpha=0.25)
        axis.legend(loc="upper left", fontsize=8)

        # Fixed bounds make expansion/contraction visible without camera motion.
        xs = [p.x for e in entries for p in e.particles]
        ys = [p.y for e in entries for p in e.particles]
        xs.extend(e.reader.position.x for e in entries)
        ys.extend(e.reader.position.y for e in entries)
        if reference is not None:
            xs.extend(e.trueTag.position.x for e in reference)
            ys.extend(e.trueTag.position.y for e in reference)
        margin = max(max(xs) - min(xs), max(ys) - min(ys), 1.0) * 0.08
        axis.set_xlim(min(xs) - margin, max(xs) + margin)
        axis.set_ylim(min(ys) - margin, max(ys) + margin)
        readerX = [e.reader.position.x for e in entries]
        readerY = [e.reader.position.y for e in entries]
        estimateX = [e.estimatedTag.position.x if e.estimatedTag else float("nan") for e in entries]
        estimateY = [e.estimatedTag.position.y if e.estimatedTag else float("nan") for e in entries]
        trueX = [e.trueTag.position.x for e in reference] if reference is not None else []
        trueY = [e.trueTag.position.y for e in reference] if reference is not None else []

        def draw(index: int):
            e = entries[index]
            cloud.set_offsets(np.array([(p.x, p.y) for p in e.particles]).reshape(-1, 2))
            count = len(e.particles)
            cloud.set_array(np.array([p.weight * count for p in e.particles]))
            readerTrail.set_data(readerX[:index + 1], readerY[:index + 1])
            estimateTrail.set_data(estimateX[:index + 1], estimateY[:index + 1])
            readerMarker.set_data([readerX[index]], [readerY[index]])
            estimateMarker.set_data([estimateX[index]], [estimateY[index]])
            if reference is not None:
                trueTrail.set_data(trueX[:index + 1], trueY[:index + 1])
                trueMarker.set_data([trueX[index]], [trueY[index]])
            measurement = ("No sample due" if e.measurement is None else
                           "Valid RFID" if e.measurement.isValid else "Invalid RFID")
            title.set_text(f"Particle cloud | t = {e.time:.3f} s | Frame {index + 1}/{len(entries)}")
            resampling = "yes, after this snapshot" if e.resampled else "no"
            status.set_text(
                f"{count} particles | Effective sample size: {e.effectiveSampleSize:.1f} | {measurement}\n"
                f"Resampling: {resampling} | Cloud shown before resampling"
            )

        dt = entries[1].time - entries[0].time if len(entries) > 1 else 0.1
        animation = FuncAnimation(fig, draw, frames=len(entries), interval=max(dt * 1000, 1), repeat=False)
        _saveAnimation(
            animation, filename, "RFID particle cloud",
            "Play or drag the slider to inspect every timestep. Brighter particles have "
            "higher weight. A color value of 1 means uniform weight; values above 5 share "
            "the brightest color. Positions are in meters.",
        )
    finally:
        plt.close(fig)


def animateSimulation(logger: DataLogger, filename: str | Path, title: str = "EKF tracking") -> None:
    """Animate a run's truth, reader, estimate, and nominal 95% covariance ellipse."""
    from math import atan2, degrees, isfinite, log, sqrt

    import matplotlib
    matplotlib.use("Agg")
    import numpy as np
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Ellipse

    if Path(filename).suffix.lower() != ".html":
        raise ValueError("Motion animation filename must end in .html")
    if not logger.entries:
        raise ValueError("No simulation data to animate")
    entries = logger.entries
    truth = np.array([(e.trueTag.position.x, e.trueTag.position.y) for e in entries])
    readers = np.array([(e.reader.position.x, e.reader.position.y) for e in entries])
    estimates = np.array([(e.estimatedTag.position.x, e.estimatedTag.position.y)
                          if e.estimatedTag is not None else (np.nan, np.nan) for e in entries])
    ellipses = []
    bounds = [*truth, *readers, *estimates[np.all(np.isfinite(estimates), axis=1)]]
    scale = -2 * log(0.05)  # Chi-square(2) 95% quantile.
    for e in entries:
        d = e.diagnostics
        covariance = np.array([[d.covarianceXX, d.covarianceXY], [d.covarianceXY, d.covarianceYY]])
        if e.estimatedTag is None or not np.all(np.isfinite(covariance)):
            ellipses.append(None)
            continue
        values, vectors = np.linalg.eigh(covariance)
        if values.min() < -1e-10:
            ellipses.append(None)
            continue
        width, height = 2 * np.sqrt(scale * np.maximum(values[::-1], 0))
        angle = degrees(atan2(vectors[1, -1], vectors[0, -1]))
        ellipses.append((width, height, angle))
        # Include the full ellipse in fixed plotting bounds at every timestamp.
        x, y = e.estimatedTag.position.x, e.estimatedTag.position.y
        rx, ry = sqrt(scale * max(d.covarianceXX, 0)), sqrt(scale * max(d.covarianceYY, 0))
        bounds.extend(((x - rx, y - ry), (x + rx, y + ry)))
    bounds = np.asarray(bounds)
    margin = max(float(np.ptp(bounds[:, 0])), float(np.ptp(bounds[:, 1])), 1) * 0.08
    fig, axis = plt.subplots(figsize=(8, 6), dpi=90)
    try:
        fig.subplots_adjust(bottom=0.22, top=0.86)
        trails, markers = [], []
        for color, marker, label in (("tab:red", "*", "True tag"),
                                     ("tab:blue", "s", "Reader"), ("tab:orange", "x", "Estimate")):
            trail, = axis.plot([], [], color=color, alpha=0.4)
            point, = axis.plot([], [], marker, color=color, markersize=9, label=label)
            trails.append(trail)
            markers.append(point)
        ellipse = Ellipse((0, 0), 0, 0, facecolor="tab:orange", edgecolor="tab:orange",
                          alpha=0.2, label="Nominal 95% position ellipse")
        axis.add_patch(ellipse)
        heading = fig.suptitle(title)
        status = fig.text(0.10, 0.06, "", fontsize=9)
        axis.set(xlabel="x (m)", ylabel="y (m)", aspect="equal",
                 xlim=(bounds[:, 0].min() - margin, bounds[:, 0].max() + margin),
                 ylim=(bounds[:, 1].min() - margin, bounds[:, 1].max() + margin))
        axis.grid(alpha=0.25)
        axis.legend(loc="upper left", fontsize=8)

        def draw(index):
            e = entries[index]
            for path, trail, marker in zip((truth, readers, estimates), trails, markers):
                trail.set_data(path[:index + 1, 0], path[:index + 1, 1])
                marker.set_data([path[index, 0]], [path[index, 1]])
            parameters = ellipses[index]
            ellipse.set_visible(parameters is not None)
            if parameters is not None:
                ellipse.center = tuple(estimates[index])
                ellipse.width, ellipse.height, ellipse.angle = parameters
            error, separation = logger._errors(e)
            errorText = f"{error:.3f} m" if isfinite(error) else "waiting for acquisition"
            reading = "No sample due" if not e.measurementDue else "Valid RFID" if e.measurement.isValid else "Invalid RFID"
            heading.set_text(f"{title} | t = {e.time:.3f} s | Frame {index + 1}/{len(entries)}")
            status.set_text(f"Position error: {errorText} | Reader/tag separation: {separation:.3f} m\n"
                            f"Position uncertainty: {e.diagnostics.positionUncertainty:.3f} m | {reading}\n"
                            "Ellipse is nominal Gaussian uncertainty, not guaranteed coverage.")

        dt = entries[1].time - entries[0].time if len(entries) > 1 else 0.1
        animation = FuncAnimation(fig, draw, frames=len(entries), interval=max(dt * 1000, 1), repeat=False)
        _saveAnimation(animation, filename, title,
                       "Play, pause, step, or drag the slider through the saved run. "
                       "The ellipse shows nominal 95% position uncertainty under a Gaussian approximation. "
                       "An inaccurate filter can be overconfident; compare the ellipse with the true tag.")
    finally:
        plt.close(fig)


def _saveAnimation(animation, filename: str | Path, title: str, description: str) -> None:
    """Embed Matplotlib frames and offline playback controls in one HTML file."""
    import re
    from html import escape
    import matplotlib

    # Embed all frames rather than silently truncating at Matplotlib's default
    # 20 MB limit. Export size grows with run length and particle count.
    with matplotlib.rc_context({"animation.embed_limit": float("inf")}):
        player = animation.to_jshtml(default_mode="once")
    # Matplotlib's controls normally load Font Awesome from a CDN. Replace
    # the icons with text so the saved animation also works fully offline.
    player = re.sub(r'<link\b[^>]*>', '', player)
    icons = {"minus": "−", "fast-backward": "|◀", "step-backward": "◀|",
             "play": "▶", "pause": "Ⅱ", "step-forward": "|▶",
             "fast-forward": "▶|", "plus": "+"}
    iconCSS = "\n".join(f'.fa-{name}::before {{ content: "{label}"; }}' for name, label in icons.items())
    document = (
        '<!doctype html><html lang="en"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width, initial-scale=1">'
        f'<title>{escape(title)}</title><style>'
        'body { margin: 24px auto; max-width: 900px; padding: 0 16px; '
        'font: 16px system-ui, sans-serif; color: #20242b; background: #f5f7fa; }'
        '.animation { background: white; border-radius: 12px; padding: 12px; }'
        '.animation img { max-width: 100%; height: auto; }'
        '.fa { font-style: normal; } .fa-flip-horizontal { display: inline-block; transform: scaleX(-1); }'
        + iconCSS + f'</style></head><body><h1>{escape(title)}</h1>'
        f'<p>{escape(description)}</p>'
        + player + '</body></html>'
    )
    Path(filename).write_text(document, encoding="utf-8")
