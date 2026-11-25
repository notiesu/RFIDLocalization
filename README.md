# RFID controls prototype (Python)

Create a MATLAB project from the existing `matlab/` folder. Project metadata is
left for MATLAB to initialize locally; see the instructions below.

The [MATLAB assignment implementation](matlab/README.txt) contains one particle
filter, baseline/final controllers, the six Simulink subsystems, and required
scenario analyses, metrics, and plots. See its README for commands and remaining
manual deliverables.

Python 3.10+. The project has a local `.venv` environment using Python 3.10.20,
including NumPy for the EKF and Matplotlib for plotting. From this directory, activate it:

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

Compare saved runs directly in [notebooks/compare_estimators.ipynb](notebooks/compare_estimators.ipynb).
Select the project's `.venv` Python kernel and run all cells. The defaults cover
all three assignment cases, comparing the particle filter, EKF monitoring, and
EKF rejection, with sensor seed 42. Edit `CASE_RUNS` to choose labeled main output
CSV paths or add methods/settings; use `PLOT_CASES` to limit plotted cases.
The notebook displays metric tables, trajectories, error, uncertainty,
reader separation, valid-read availability, RSSI/bearing measurements, speed,
turn rate, heading, and recovery-event details inline. It checks that ground truth
and timestamps match and that each summary belongs to its CSV. Keep each
`.summary.json` alongside its CSV. Executed inline results for all nine runs are
included, together with NIS and rejected-update diagnostics. Within-case paths
are checked; distinct cases are kept separate.

Notebook dependencies are already installed in this environment. To recreate them:

```sh
python -m pip install -r requirements-notebook.txt
```

Edit `METRICS` in the notebook to choose the summary rows. Outputs appear inside
the notebook; it does not export a separate dashboard. The comparisons are of
closed-loop runs, so reader paths and measurements can differ between methods.

Recreate the nine output pairs from the project root:

```sh
for scenario_name in straight turning abrupt_maneuver; do
  python -m controls --scenario "scenarios/assignment/${scenario_name}.csv" --estimator particle --seed 42 --output "output/assignment/${scenario_name}_particle.csv"
  python -m controls --scenario "scenarios/assignment/${scenario_name}.csv" --estimator ekf --ekf-gate monitor --seed 42 --output "output/assignment/${scenario_name}_ekf_monitor.csv"
  python -m controls --scenario "scenarios/assignment/${scenario_name}.csv" --estimator ekf --ekf-gate reject --seed 42 --output "output/assignment/${scenario_name}_ekf_reject.csv"
done
```

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

Generate stochastic random-walk scenarios:

```sh
python controls/scenarios/randomwalk.py
python -m controls --scenario scenarios/randomwalk/gaussian.csv --output output/gaussian.csv
python -m controls --scenario scenarios/randomwalk/student_t.csv --output output/student_t.csv
```

`gaussian.csv` uses independent Gaussian increments; `student_t.csv` uses
heavy-tailed Student-t increments with 3 degrees of freedom. Both start at `(5,5)`
and have zero-mean per-axis increments with variance `diffusion² * dt`. The
Student-t draws are standardized to match that theoretical variance, not adjusted
to match each realized sample variance. Defaults are 20 s, `dt=0.1`,
`diffusion=0.3 m/sqrt(s)`, and seed 42. The walk has no drift; position is the
cumulative sum of increments. Velocities in the CSV describe the outgoing interval;
the last row repeats the last interval's velocity. Paths are not clipped or
reflected, so other seeds/settings can leave the 10x10 m workspace. The generator
accepts `--duration`, `--dt`, `--diffusion`, `--seed`, and `--output-dir`.
These position-increment walks differ from the estimators' constant-velocity
motion assumption, so they also test motion-model mismatch.

Choose the estimator (the particle filter remains the default):

```sh
python -m controls --scenario scenarios/assignment/straight.csv --estimator ekf --output output/ekf.csv --plot output/ekf.png
```

`--estimator particle` selects `ParticleFilterEstimator`; `--estimator ekf`
selects `EKFEstimator`. Both use the common estimator interface and existing
simulation CSV, summary JSON, uncertainty metrics, and static trajectory plots.
`--particle-log` and `--particle-animation` require the particle estimator;
`--particles` applies only to it. The EKF is deterministic for a given measurement
sequence; `--seed` still controls sensor noise/dropouts.

`--animation output/run.html` exports an offline motion animation for either
estimator. The particle filter includes its particle cloud; the EKF includes a
nominal 95% Gaussian position covariance ellipse. Both show true/estimated tag
and reader paths with playback and per-frame controls. The ellipse is based on
the filter's covariance and does not guarantee actual coverage.

```sh
python -m controls --scenario scenarios/randomwalk/gaussian.csv --estimator particle --seed 42 --output output/randomwalk/gaussian_particle.csv --animation output/randomwalk/gaussian_particle_seed42.html
python -m controls --scenario scenarios/randomwalk/gaussian.csv --estimator ekf --seed 42 --output output/randomwalk/gaussian_ekf.csv --animation output/randomwalk/gaussian_ekf_seed42.html
```

The EKF estimates `[x,y,vx,vy]` with a Gaussian mean/covariance and the same
constant-velocity transition as the particle filter. Its process covariance is
`accelerationStdDev² * G @ G.T`, where `G` applies one constant x/y acceleration
over a tick. RSSI and relative-bearing Jacobians linearize the clamped sensor
model, bearing innovations are wrapped, and a Joseph covariance update and linear
solve avoid a direct matrix inverse. Invalid/missing reads trigger prediction only.
Without an initial prior, the first valid read supplies approximate polar-to-Cartesian
initialization and is not assimilated twice. Inside the RSSI plateau, initialization
uses radial disk moments; this is a Gaussian approximation, not an exact posterior.
Velocity starts at zero with 1 m/s standard deviation. Explicit priors accept a
finite, symmetric positive-semidefinite 4x4 `initialCovariance` (default identity).

```python
from controls import EKFEstimator, RFIDConfig

estimator = EKFEstimator(RFIDConfig())
# estimator.update(measurement, reader, dt)
# estimator.covariance is the full 4x4 covariance; diagnostics expose position covariance.
```

The EKF is a local Gaussian approximation. Within `referenceDistance`, RSSI's
range derivative is zero; at coincident positions bearing is undefined and the
measurement correction is skipped. Near a reader/tag crossing the local bearing
linearization can become inaccurate and the filter can diverge while reporting
small covariance. This occurs with the current pursuit controller on the supplied
paths; the default seeded abrupt-maneuver run has about 1.41 m RMSE. Fixed-reader
tests on all three paths have about 0.38–0.41 m RMSE. These are simulation results,
not a claim that either estimator is uniformly better. EKF ESS is not applicable
and is logged as `nan`; resampling is false.

EKF innovation checks are independent of the motion model. Before a correction,
the filter computes `NIS = residual.T @ solve(S, residual)`, where `S = H P H.T + R`.
The bearing residual is wrapped. The default threshold is 9.21034, the nominal
99% chi-square threshold for two measurement components under a correctly
specified local Gaussian model. Configure `--ekf-gate-threshold` or the estimator's
`innovationGateThreshold` argument. Three `--ekf-gate` / `innovationGateMode` values
are supported:

- `monitor` (default): flag an inconsistent innovation, but keep the original EKF
  update. This preserves existing estimation behavior while making failures visible.
- `reject`: skip a flagged correction and retain the predicted mean and covariance.
  The rejected read does not shrink covariance. Prediction still occurs normally.
- `off`: calculate NIS for inspection without applying an outlier threshold.

Bootstrap initialization has no prior to check, so its NIS is unavailable. Missing
or invalid reads have no innovation test. Coincident geometry skips correction
with reason `singular_geometry`. Each tick resets the innovation diagnostics so
old rejections are not carried into missing-sample ticks. CSV fields include RSSI
and bearing residuals, NIS, gate threshold/mode, outlier flag, acceptance flag, and
rejection reason. Summaries report all pre-decision NIS values and rejection counts.
Sensor validity remains separate: a valid RFID read can be rejected by the EKF.

Rejection alone is not a recovery algorithm. On the seed-42 Gaussian walk it
increases RMSE from 0.830 m to 1.748 m by rejecting information after a bad prior;
on the assignment cases it has mixed results. For that reason monitoring is the
default and rejection is an explicit experiment. No scenario-specific process
model, covariance inflation, or automatic reinitialization has been added.

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
to reproduce the verified package versions. NumPy is a runtime dependency;
Matplotlib is needed for plots and particle animations.

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
| `EKFestimator.py` | Extended Kalman filter with RSSI/bearing linearization |
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

## Instrumented trajectory generation

For an entirely in-memory workflow, open
[`notebooks/generate_trajectory.ipynb`](notebooks/generate_trajectory.ipynb) and
run all cells. It keeps instruments, inputs, the trajectory, and evaluation truth
in Python variables and displays plots inline. Seeds recreate the data after a
kernel restart; no intermediate CSV is required.

Generate the experiment's instruments separately, then pass them into the
trajectory generator (run from the project directory):

```sh
.venv/bin/python -m controls.scenarios.instruments --output scenarios/iv/instruments.csv --duration 20 --dt 0.1 --seed 42
.venv/bin/python -m controls.scenarios.instrumented --instruments scenarios/iv/instruments.csv --output scenarios/iv/trajectory.csv --seed 42
```

The instrument module supplies independent Gaussian `z_x,z_y` values. Replace
that module or supply your own uniformly spaced `time,z_x,z_y` CSV to change the
external instrument process. Each row drives one outgoing interval; 200 instrument
rows at dt=0.1 produce 201 trajectory states ending at 20 seconds.

The trajectory generator implements exactly:

```text
u_t = alpha*z_t + delta*w_t + nu_t
position_next = position + velocity*dt
velocity_next = velocity + (beta*u_t + gamma*w_t)*dt
```

Defaults are alpha=1, beta=0.5, gamma=0.3, delta=0.8, independent Gaussian nu
with sigma=0.2, and initial state (2,2,0,0). CLI flags expose all coefficients
and `--input-noise-stddev`. The Python API also accepts `initialState`.
The same w draw appears in both equations. A random two-component Gaussian
mixture defines w; its parameters are drawn once per trajectory, then independent
samples are drawn across time and axes. It has mean zero and unit variance,
fixing the ambiguity between disturbance scale and gamma. `--distribution-seed`
holds this law fixed while `--seed` changes its realizations. RNG streams are
separate from the instrument generator even with matching numeric seeds.

The primary CSV contains `time,x,y,vx,vy,u_x,u_y,z_x,z_y`; the last row has no
outgoing inputs. It works with the existing scenario loader, which ignores extra
columns. The matching `trajectory.truth.json` holds hidden coefficients,
distribution parameters, and w/nu draws **for offline evaluation only**. Existing
PF/EKF runs still estimate state; they do not consume u/z or learn these parameters.

This initial law is stationary, not yet state/time dependent. Trajectories have
no bounds or speed clipping, which would alter the proposed equations. This is
a discrete-time acceleration model: noise enters velocity multiplied by dt,
so changing dt changes its accumulated variance; it is not Brownian scaling.
External instruments must remain independent of hidden disturbances for the
intended IV interpretation. Later inference must account for the dependence of
w on observed u; treating the residual as independent of u would miss the
endogeneity this generator intentionally creates.

## Beta parameter inference

Open [`notebooks/compare_beta_estimators.ipynb`](notebooks/compare_beta_estimators.ipynb)
and run all cells for an in-memory comparison of beta estimates, posterior
intervals, and final errors against ground truth. It includes a shared-disturbance
case and a delta=0 control with matched instruments and noise seeds.

`ParameterParticleFilterEstimator` derives from the generic `Estimator`, since
the existing tracking particle filter is specific to RFID measurements and tag
states. It learns a static scalar beta from `ParameterObservation(acceleration,
inputs, instruments)`, pooling axes. Acceleration is an observation here; it is
not inferred from RFID. The notebook computes acceleration from simulated
velocities and adds independent Gaussian observation error, shared across methods.

The estimator accepts no generator parameters, mixture law, latent draws, or true
observation/process-noise scales. Both particle likelihoods learn their nuisance
parameters jointly with beta:

- `independent`: `a = beta*u + e`, learning beta and a Gaussian residual scale.
  This deliberately ignores input/disturbance correlation.
- `shared`: `u = alpha*z + r; a = beta*u + lambda*r + e`, learning beta, alpha,
  lambda, and two residual scales through the joint working likelihood p(u,a|z).
  Lambda models residual correlation; it is not the hidden gamma or delta.

The generator remains non-Gaussian; these are Gaussian working approximations,
not recovery of its exact mixture distribution. Fixed `ParameterPrior` bounds
are inference assumptions independent of generator metadata. The beta prior is
uniform on [-3,3]; scale priors are log-uniform on [0.01,10]. Priors are not set
using true beta or IV estimates. The notebook's `infer_beta` function accepts
only observations, timestep, inference seed, particle count, and priors. It never
reads `result.truth`. True beta is used only for generation and evaluation.
Simulated acceleration measurements are produced from true velocity changes
plus sensor noise during observation generation; no true velocities reach
inference. This does not yet estimate hidden states from RFID.

Particles have a bounded uniform beta prior and no artificial parameter drift.
Low ESS triggers systematic resampling and Metropolis moves targeting the full
accumulated posterior, following [static-parameter resample/move SMC](https://academic.oup.com/biomet/article-abstract/89/3/539/251804).
`history` retains posterior means, standard deviations, approximate 95% credible
intervals, ESS, and move acceptance. `particles` and `weights` return copies of
the marginal beta cloud; `parameterParticles` and `parameterNames` expose the
joint learned parameters. Sufficient statistics summarize only received
observations, allowing moves without storing or accessing future observations.

`InstrumentalVariableEstimator` provides a running, zero-intercept, common-beta
estimate `sum(z*a)/sum(z*u)` and the pooled first-stage coefficient. It requires
no noise-law parameters. Near-zero cross-moments return no estimate; early/weak
instrument estimates can still be unstable and are not clipped to the PF prior.
Its history is a point-estimate history, not a Bayesian posterior interval.

Repeat across seeds and dependence strengths before making a performance claim.
A narrow posterior under a misspecified likelihood is not proof of correctness.
The learned shared model provides a baseline for handling residual correlation;
a combined PF+IV estimator is not implemented yet. Gaussian posterior intervals
are model-based and are not guaranteed to have nominal coverage under the
non-Gaussian generator. The delta=0 control now compares differently parametrized
learned models, so their finite-sample posteriors need not be identical.

## Joint state and beta inference

[`notebooks/joint_state_parameter_estimation.ipynb`](notebooks/joint_state_parameter_estimation.ipynb)
estimates x,y,vx,vy and beta from the same prerecorded RFID/input stream. It
compares ignoring confounding against learning an instrument-based residual
relationship, with original readings and a 5–7 s read outage. Plots show beta
uncertainty, tracking error, next-velocity prediction error, and the joint
position/beta particle cloud. Tables include state/prediction RMSE and move
acceptance; this single run is illustrative, not evidence of general superiority.

`JointParticleFilterEstimator` takes `RFIDMeasurement` plus
`JointFilterContext(reader, inputs, instruments)`. For an update at t[i+1],
supply u[i],z[i], the inputs on the interval ending at that reading. Initialize
at t=0 with `predict=False` and no inputs. The return value is a `TagState`;
`history[-1]` also contains beta, its interval, and parameter ESS. Motion/noise
parameters are learned, not passed from generator truth. Known RFID calibration
remains a measurement-model assumption. True motion is used only by the
simulator/evaluation layer, not by inference. Initial localization is an
empirical prior from the first valid reading, with a generic velocity prior.

`predictNextState(context, dt)` forecasts state mean/covariance without consuming
a future reading or mutating inference. Here the context contains the outgoing
input. `lastPrediction` retains the forecast made before the current update's
RFID reading, allowing evaluation without fitting that same reading first.

Each parameter hypothesis has its own motion particle filter, following the
[SMC² nested-filter approach](https://arxiv.org/abs/1101.1528). Static parameters
are rejuvenated with particle marginal Metropolis moves that replay received
observations; beta is not assigned artificial process noise. Both methods assume
constant unknown beta per run, as does the current generator. That is a declared
model assumption, not knowledge of beta's true value. Time-varying beta is not
tested here. Gaussian motion likelihoods remain working approximations.

Fixed particle counts, potentially low move acceptance, and observation-history
replays limit long-run performance/cost. Inspect acceptance and repeat/increase
particle counts before trusting precise parameter posteriors. This notebook uses
a fixed reader and shared sensor records; the existing pursuit `Simulation` and
CLI do not supply instrument/input contexts and remain unchanged.

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
