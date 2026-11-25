ACSL RFID TRACKING — ASSIGNMENT IMPLEMENTATION

Team members/contributions: fill in actual names and roles.
Required products: MATLAB and Simulink. Target release: R2026b.

CREATE THE PROJECT IN MATLAB
Close any old project. Select Home > New > Project (older releases: From Folder).
Name it RFIDControls and select this existing matlab/ folder as its location.
Let MATLAB create its own project definition and metadata.
The project root must be this folder, not its parent ControlsProject/ folder.
Run setupControls in the Command Window each session to add this folder and
simulink/ to the path. Do not recursively add +controls to the path.
The folder is supplied before its first run: main generates models/ and output/.

RUN ONE EXTERNAL CSV SCENARIO
With this folder selected as MATLAB's current folder:
  setupControls
  main('scenarios/assignment/straight.csv')
  main('scenarios/assignment/turning.csv','controllerMode',2)
  main('scenarios/assignment/abrupt_maneuver.csv','controllerMode',0)

Controller modes: 0=fixed-reader baseline, 1=proportional pursuit, 2=final design.
The final design maintains a 2 m estimated standoff, slows as particle spread
increases, and stops forward motion while RFID availability is lost. It still
turns toward the estimated tag. No integral action is used.

RUN REQUIRED ASSIGNMENT ANALYSES
  results=runAssignmentCases;
Runs all three paths with fixed/baseline/final control, plus increased RSSI
noise and increased dropout with final control. These are assignment scenarios,
not a software test suite. Outputs: per-run CSV/MAT/PNG and comparison.csv.
CSV logs include true/estimated/reader trajectories, RSSI/bearing, validity,
uncertainty, errors, reader motion and commands. MAT files retain all entries,
configuration and required metrics, including recovery episodes.

MODEL
Generated file: models/rfid_tracking.slx.
  buildSimulinkModel('scenarios/assignment/straight.csv');
  load_system('models/rfid_tracking.slx'); sim('rfid_tracking');
The six named subsystems match ControlModel.jpg. A visible constant selects
fixed/baseline/final control. All use the same MATLAB component classes.
The model uses normal-mode Level-2 MATLAB S-functions, not generated code.
main uses this Simulink model; there is no second standalone simulation loop.
Rebuilding replaces the closed generated model. An open model is preserved.

MODELS AND DEFAULTS
Tag state: x,y,vx,vy. Reader state: x,y,heading,velocity,turnSpeed.
Units: m, s, rad; commands are turn rates in rad/s (not turn angles).
Workspace: 10 x 10 m, no excluded regions. CSV trajectory starts at t=0 and
has uniform positive spacing; columns are time,x,y,vx,vy, optional event.
The three supplied trajectories start at (2,2), reader at (0,0), last 20 s,
and use dt=0.1 s. Their categories are prescribed; exact paths are our choices.

RFID: RSSI at 1 m=-40 dBm, path-loss exponent=2, RSSI sigma=2 dB,
bearing sigma=0.1 rad, range<=10 m, RSSI>=-80 dBm, 10 Hz, dropout=5%.
RSSI clamps inside 1 m. Inferred range is noisy/nonlinear and can be biased.
Only RFID and reader state enter the particle filter; true tag state reaches
only sensing and offline logging. Before first detection, the reader waits.

Estimator: 500 particles, CV with Gaussian acceleration sigma=0.5 m/s^2 per
axis. Each tick uses position+=velocity*dt+0.5*acceleration*dt^2 and
velocity+=acceleration*dt. Initialize position from the first valid RSSI/bearing,
sampling sensor uncertainty; initial per-axis velocity prior is N(0,1).
Weight using Gaussian RSSI and wrapped-bearing residuals with the stated
sensor covariance. Missing/invalid readings cause prediction only.
Systematic resampling occurs below 50% effective sample size. Reported
uncertainty is weighted RMS position spread before resampling, not true error.

Controller: Kv=0.5, Kw=2, speed<=2 m/s, turn rate<=1.5 rad/s.
Reader plant: midpoint-heading unicycle integration, ideal command response.
Final controller: standoff=2 m, uncertainty speed scale=1 m. These values
are model/design assumptions, not validated hardware specifications.

PROVISIONAL REQUIREMENTS (not numerical thresholds supplied by the assignment)
Position RMSE<=0.5 m; valid RFID time>=95%; speed<=2 m/s; |turn|<=1.5 rad/s;
recovery<=3 s. Recovery means error<=0.5 m for 1 continuous second with RFID
available. Explicit events start at their CSV timestamp; dropout recovery
starts at the next valid read. comparison.csv reports compliance from actual
results. Unrecovered/interrupted episodes are not counted as successes.

SCOPE
One estimator only (PF), baseline/final controls, required scenario analyses,
metrics and static plots. No EKF, generic estimator framework, random walks,
particle-history export, HTML animation, JSON layer or software test suite.
Hardware/BOM, slides and actual team contributions remain manual deliverables.
MATLAB execution/model generation are still unverified here because local
MATLAB startup/license checkout fails. No numerical results are fabricated.
