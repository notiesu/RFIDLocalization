# RFID controls prototype (Python)

Python 3.10+. The project has a local `.venv` environment using Python 3.10.20,
including Matplotlib and its dependencies. From this directory, activate it:

```sh
source .venv/bin/activate
```

To recreate an environment on another machine:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Run the prototype and tests inside the activated environment:

```sh
python -m controls --scenario scenarios/my_run.csv --output output/my_run.csv
python3 -m unittest discover -s tests -v
```

Scenarios are generated externally. `--scenario` is a required CSV path; there
are no built-in trajectories. The simulation runs every row, deriving its timestep
and duration from the CSV. Sensor defaults are 10 Hz measurements, 5% dropouts,
and 500 estimator particles. Use `--help` for options.

Ground-truth CSV format:

```csv
time,x,y,vx,vy
0.0,4.0,2.0,0.5,0.15
0.1,4.05,2.015,0.5,0.15
0.2,4.1,2.03,0.5,0.15
```

Each row supplies the true tag state: `time` in seconds, `x,y` in meters, and
`vx,vy` in meters/second. Include a header and at least two rows. Timestamps must
start at zero and increase with uniform spacing, since the simulation uses a
fixed timestep. All five values must be finite numbers. Column order is flexible;
an optional `event` column marks disturbances for logging; other extra named
columns are ignored. States and velocities are read as supplied,
without interpolation or automatic velocity calculation. The final timestamp
defines the run duration; `--dt` and `--duration` are no longer CLI options.
Use `--update-rate` at or below the CSV sample rate. The reader starts at the
origin; ground truth describes the tag, not an imposed reader trajectory.

Generate the assignment's three required trajectory categories (page 4):

```sh
python controls/scenarios/assignment.py
python -m controls --scenario scenarios/assignment/straight.csv --output output/straight.csv --particle-animation output/straight.html
```

The generator writes only these CSVs to `scenarios/assignment/`:

| File | Motion | Purpose |
| --- | --- | --- |
| `straight.csv` | East from `(2,2)` to `(8,2)` | Nominal constant-velocity tracking |
| `turning.csv` | Smooth 90° left arc from `(2,2)` to approximately `(5.82,5.82)` | Tracking during continuous turning |
| `abrupt_maneuver.csv` | East to `(5,2)`, then abruptly north at 10 s, ending at `(5,5)` | Recovery from an unexpected velocity-direction change |

All three use a `[0,10] × [0,10]` m workspace, a 20 s duration, a 0.1 s timestep,
and 0.3 m/s tag speed. The reader starts at `(0,0)`. For the abrupt maneuver,
position is continuous and the new northward velocity applies at exactly 10 s.
The exact paths, speeds, and initial conditions are our test choices; the
assignment prescribes the categories. An abrupt maneuver is one of its allowed
challenge cases, so a dropout zone or obstacle is not needed for this third path.
Re-run the generator to replace these files, or set `--output-dir` to write
elsewhere. The simulator still only consumes externally generated CSVs.

This supplies the trajectory set only. Page 5 separately requires fixed-reader,
baseline-controller, and final-controller comparisons, plus at least two
non-nominal robustness tests; those are not replaced by these three trajectories.

Optional plotting:

```sh
python -m controls --scenario scenarios/my_run.csv --output output/my_run.csv --plot output/my_run.png
```

Particle-cloud animation and logging:

```sh
python -m controls --scenario scenarios/my_run.csv --output output/my_run.csv --particle-log output/particles.csv --particle-animation output/particles.html
open output/particles.html
```

The HTML file contains every timestep and works offline. Use play/pause, the
slider, or the previous/next frame buttons to inspect the cloud. The reader,
estimated tag, and true tag are overlaid with their paths. Particle color shows
weight divided by uniform weight (`particleCount * weight`): 1 means uniform,
and values above 5 share the brightest color. Each frame shows its timestamp,
effective sample size, measurement status, and whether resampling followed.
Exporting the animation renders an image for every tick; long runs produce larger
HTML files and take longer to export.

`estimator.logger.entries` automatically stores one snapshot per call to
`ParticleFilterEstimator.update()`, including empty clouds before acquisition. Simulation
timestamps align exactly with its main log. Snapshots contain particle position,
velocity, weight, estimated tag state, reader state, measurement (or `None` if no
sample was due), effective sample size, and the resampling flag. The cloud is
captured **after weighting, before resampling**, so its weighted mean matches the
reported estimate and important weight differences remain visible. Later filter
updates cannot mutate past snapshots. Ground truth is added only by the visualizer
from the simulation log; the estimator still cannot access it.

The particle CSV has one row per particle per timestep. Empty clouds get one row
with blank particle fields. `particle_index` identifies a slot within a frame,
not a persistent identity across resampling. For direct use:

```python
from controls import ParticleFilterEstimator, EstimatorLogger, RFIDConfig

estimator = ParticleFilterEstimator(RFIDConfig())
# Call estimator.update(measurement, reader, dt) each timestep.
history = estimator.logger.entries
estimator.logger.exportCSV("particles.csv")
estimator.logger.clear()
```

History uses memory proportional to timesteps times particle count. To disable
it for longer runs, pass `logger=EstimatorLogger(enabled=False)` to `ParticleFilterEstimator`.
The logger does not consume random numbers or change filtering results. Low-level
calls such as `predictParticles()` and `updateWeights()` do not create snapshots;
use `update()` once per timestep. Direct calls default to times `0, dt, 2*dt, ...`;
pass `time=...` explicitly for another clock. No new dependencies are needed.

Use `deactivate` to leave the environment. `.venv/` is excluded from Git;
`requirements.txt` records the project dependency. `requirements-lock.txt` records
all installed dependency versions; with Python 3.10, install that file instead
to reproduce the verified package versions. The simulation, CSV logging, and
tests use only the Python standard library.

## Relationship to the original code

`controls/` is the runnable Python prototype. `cpp/`, the MATLAB `src/` scaffolding,
and `ControlModel.jpg` are retained as references. Python module names, class
names, public method names, and struct fields follow the C++ files deliberately.

| Python module | Source / behavior |
| --- | --- |
| `Utils.py` | C++ shared structs as dataclasses; one canonical `Coord` definition |
| `RFIDSensor.py` | Original range/dropout checks, clamped log-distance RSSI, sensitivity check, and relative bearing |
| `ReaderPlant.py` | Original midpoint-heading integration and wrapped heading |
| `ScenarioGenerator.py` | External ground-truth CSV loading, stored-state iterator, and reset |
| `DataLogger.py` | Original diagnostics, RMSE, tracking distance, and CSV fields |
| `Controller.py` | Existing config defaults; new implementation because `Controller.cpp` was empty |
| `Estimator.py` | Generic abstract estimator interface |
| `ParticleFilterEstimator.py` | Particle-filter implementation based on the JPG |
| `EstimatorDiagnostics.py` | Shared tracking diagnostics independent of the particle logger |
| `EstimatorLogger.py` | Per-timestep weighted particle-cloud snapshots and CSV export |
| `Simulation.py`, `main.py` | Feedback loop and CLI accepting an external scenario CSV; original `main.cpp` was empty |
| `Visualization.py` | Optional trajectory/performance figure and interactive particle-cloud animation |

Python deep copies preserve C++ value semantics for plant states, scenarios,
and log entries. Random seeds make Python runs reproducible; random draws are
not bit-for-bit compatible with C++ distribution implementations. The Python
source does not depend on the duplicate/inconsistent declarations in `cpp/Utils.cpp`.

`Estimator[StateT, MeasurementT, ContextT]` is an abstract base class for sequential
estimators. It defines `initialized`, `update()`, `calculateStateEstimate()`, and
optional tracking `diagnostics`; it contains no particle-filter or RFID model.
`ParticleFilterEstimator` inherits from it and implements the existing RFID
particle filter. Instantiate `ParticleFilterEstimator(RFIDConfig(), ...)` instead
of the old `Estimator(...)`. The simulation uses the base interface, so other
estimators can be substituted without providing a particle logger. Particle
history exports and animations belong to the particle-filter implementation.

## Feedback loop and assumptions

At time `t`, the simulation retrieves the true tag state and reader state, samples
RFID if due, predicts/updates the estimator, computes a command, and logs a
snapshot. It then applies the command over the next interval. All data in a CSV
row describes the same timestamp. The final row does not advance the plant beyond
the scenario. Supplied scenario states are instantaneous states at `i * dt`.

`RFIDSensor.measure()` still performs one measurement attempt per call, just like
the C++ version. `Simulation` enforces `updateRate`, sampling at the first simulation
tick on or after each scheduled sensor time. Choose `dt` so the sensor period is
an integer multiple for exact timing; otherwise timing is quantized by less than
one timestep. A sensor rate above the simulation tick rate is rejected. Fresh
measurements are consumed once; missing samples trigger prediction only.

The particle filter maintains `[x, y, xdot, ydot]`. Each prediction draws independent
x/y accelerations (default standard deviation 0.5 m/s²), applies constant
acceleration for that tick, and retains the original weight. Joint Gaussian
RSSI/bearing likelihoods update weights in log space. Bearings use wrapped angle
residuals; a numerical width of `1e-6` avoids division by zero in noiseless tests.
Systematic resampling occurs below `0.5 * particleCount` effective particles.
The estimate is computed before resampling.

Without an explicit initial prior, the first valid measurement initializes
position particles by inverting noisy RSSI and noisy bearing, with ranges capped
at `maxRange`. This is an approximate initialization distribution, not an exact
Bayesian posterior. Velocity starts with a zero-mean 1 m/s Gaussian prior.
Inside `referenceDistance`, initialization samples a disk because the sensor's
RSSI model is flat there. No estimator or controller receives the true tag state.
Until acquisition the estimate is `None` and the reader holds still. After loss
of signal the filter predicts forward; there is no search or reacquisition policy.

The controller turns toward the estimated position:
`omega = clamp(Kw * wrappedHeadingError, ±maxTurnRate)` and
`v = min(maxVelocity, Kv * distance) * max(0, cos(headingError))`.
It turns in place for targets behind it and stops at zero distance. This is a
simple position pursuit controller, with no velocity feedforward or stopping
distance. It will generally lag a moving tag. The reader is the original ideal
kinematic plant: commanded speed and turn rate take effect immediately.

The main CSV preserves the original 22 columns, followed by `measurement_due`
and the assignment diagnostics below. Invalid samples use the original `Pr=0`
sentinel; check `valid` before interpreting RSSI. Before acquisition, estimates,
errors, and position uncertainty are `nan`. Duration is last minus first timestamp.

## Assignment logging metrics

Every CLI run writes both the per-timestep CSV and a matching summary JSON,
e.g. `output/straight.csv` and `output/straight.summary.json`. Metrics also print
in the terminal. JSON uses `null` for unavailable metrics, not a fabricated zero.

- **Estimator uncertainty:** `position_cov_xx`, `position_cov_xy`, and
  `position_cov_yy` are the weighted position covariance before resampling, in m².
  `position_uncertainty = sqrt(cov_xx + cov_yy)` is the cloud's weighted RMS
  distance from its mean in meters. This measures estimated spread, not actual
  position error or a guaranteed confidence bound. Mean uncertainty is time-weighted
  over intervals with a valid estimate; worst uncertainty is the maximum logged
  value. The summary includes uncertainty time coverage. These diagnostics are
  available even with full particle-history logging disabled. The original
  `uncertainty` column remains RSSI noise sigma in dB. ESS and resampling flags
  are also included in the main CSV.
- **Valid-read availability:** `validRate` retains the percentage of successful
  sensor attempts. `validTimeRate` reports percentage of elapsed time for which
  the latest scheduled attempt was valid. `rfid_available` holds that validity
  until the next attempt; ticks without a scheduled sample do not reset it.
- **Actuator-limit usage:** `speed_at_limit` and `turn_rate_at_limit` mark commands
  at their configured absolute limits. The summary reports time percentages at
  each limit and records the limits used. These describe command-limit usage,
  not whether an unlogged pre-clamp request exceeded a limit. With the current
  ideal plant, commands take effect immediately. Zero or unknown limits produce
  blank flags / unavailable usage percentages.
- **Separation:** per-timestep separation, mean separation, and maximum separation
  are available alongside position RMSE and maximum estimation error.
- **Recovery:** the logger groups consecutive invalid sensor attempts into dropout
  episodes and records their start and end. The dropout recovery clock starts
  at the first subsequent valid attempt. For other disturbances, add an optional
  `event` column to the scenario CSV: a nonempty label starts a recovery clock
  at that timestamp. For a sustained disturbance, mark its end to measure recovery
  afterward. The supplied abrupt-maneuver CSV marks the turn at 10 s. These
  annotations are passed only to the logging/evaluation layer.

Recovery defaults to position error ≤ **0.5 m for 1 continuous second**, with
valid-read availability throughout. Recovery time includes that confirmation
hold and is measured at simulation ticks. It resets the hold when the condition
fails. A later disturbance interrupts an unfinished recovery. Summary event
records include start/end times, confirmation time, recovery duration, and
`recovered`, `not_recovered`, `interrupted`, or `ongoing` status. No event means an
empty recovery-event list. An unresolved event has no numeric recovery duration.
The thresholds are provisional analysis settings, not assignment-specified
acceptance criteria. Override them with `--recovery-threshold` and `--recovery-hold`,
or `DataLogger(recoveryThreshold=..., recoveryHoldTime=...)` in Python.

Time percentages use each row over `[t_i, t_{i+1})`; the final row adds no
extra duration. RMSE and the original mean-separation statistic remain sample
averages. Event markers do not infer maneuvers from noisy velocity changes.

## Design review

The decomposition in `ControlModel.jpg` is suitable for this prototype. The
important points to resolve before interpreting results as hardware performance:

- **Turn units were inconsistent.** The diagram and old utility copy say
  `turn_angle`/`turnAngle`, while the plant multiplies its command by `dt`.
  The Python prototype follows the implemented `turnRate` in rad/s.
- **Bearing is an input assumption.** The design assumes a relative bearing
  measurement in addition to RSSI. Confirm how the intended reader/antenna setup
  produces it. Disabling bearing requires changing both the sensor and likelihood;
  a zero bearing is a measurement, not a missing value.
- **RSSI loses range information near the tag.** The implemented model clamps
  distance at `referenceDistance`, so RSSI cannot distinguish distances within
  that radius. Bearing still gives direction. Do not expect precise close-range
  localization from this model.
- **Timing and uncertainty need explicit meanings.** `uncertainty` is RSSI sigma
  in dB, not position variance; bearing sigma comes from `RFIDConfig`. Sensor
  and simulation clocks are separate. These are now explicit in the Python loop.
- **Dropout and tracking limits need a policy.** With no initial detection the
  reader waits indefinitely. After detection, a long outage can leave particles
  far from the real tag; this filter has no global particle injection. It also
  treats invalid measurements as uninformative rather than exploiting non-detection.
  A hardware-oriented version should define timeout/search behavior and expose
  confidence to the controller.
- **Control is idealized.** The diagram has no standoff target, acceleration
  limits, actuator delay, or obstacles. Start with this plant for algorithm checks;
  add those effects before making claims about a physical platform. The supplied
  motion/noise parameters are demo assumptions, not calibrated values.

For direct Python use, call `loadScenarioCSV(filename)` and pass the resulting
`Scenario` to `ScenarioGenerator`. You can also construct `Scenario(dt, states)`
yourself. All main components are exported from `controls`.
