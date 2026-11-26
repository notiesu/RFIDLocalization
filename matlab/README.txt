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
The saved model is models/rfid_tracking.slx. Runs keep results in memory.

GENERATE AND RUN A SCENARIO DIRECTLY
With this folder selected as MATLAB's current folder:
  setupControls
  trajectory=generateAssignmentScenario('straight');
  main(trajectory);
Click Run in the model window. After the simulation completes:
  plotResults(trackingLog);
  trackingLog.printStats();
Other names: turning, abrupt_maneuver. Calling main() defaults to straight.
main initializes and opens the model; it does not run the simulation or return
a logger. The logging subsystem creates its own DataLogger when simulation
starts and exposes it as trackingLog in the Workspace when simulation ends.
generateAssignmentScenario.m defines all three paths in MATLAB; include it in
the submission. The table is supplied directly to the Simulink scenario block
via the model workspace; no intermediate CSV is needed.

OPTIONAL EXTERNAL CSV INPUT
  main('scenarios/assignment/straight.csv')
  main('scenarios/assignment/turning.csv','controllerMode',2)
  main('scenarios/assignment/abrupt_maneuver.csv','controllerMode',0)

Controller modes: 0=fixed-reader baseline, 1=proportional pursuit, 2=final design.
The final design maintains a 2 m estimated standoff, slows as particle spread
increases, and stops forward motion while RFID availability is lost. It still
turns toward the estimated tag. No integral action is used.

RUN REQUIRED ASSIGNMENT ANALYSES
  [results,logs]=runAssignmentCases;
  results
  plotResults(logs.turning_final);
Runs all three paths with fixed/baseline/final control, plus increased RSSI
noise and increased dropout with final control. These are assignment scenarios,
not a software test suite. Returns a comparison table and a struct of per-case
loggers; it does not save files or open figures. The paths are generated directly
with generateAssignmentScenario. Log entries retain true/estimated/reader
trajectories, measurements, validity, uncertainty, commands and event labels.
trackingLog.stats() returns the required metrics, including recovery episodes.
plotResults(trackingLog) opens six separate figures: trajectories, localization
quality, RSSI, valid reads, speed, and turn rate.

OPTIONAL SAVING
Nothing is automatically saved by main or by the logger when using GUI Run.
After a GUI run, the logger is available as trackingLog in MATLAB's Workspace:
  plotResults(trackingLog);
  trackingLog.printStats();
Use MATLAB's figure Save/Export tools to save plots. Explicit data export:
  trackingLog.exportCSV('tracking.csv');
  save('tracking.mat','trackingLog');
  writetable(results,'comparison.csv');
The former main options 'output' and 'plot' have been removed; call plotting or
export explicitly. Existing result files are left alone.

MODEL
Source of truth: models/rfid_tracking.slx. Include this file in the submission.
  main('scenarios/assignment/straight.csv');
  open_system('models/rfid_tracking.slx');
The six named subsystems match ControlModel.jpg. A visible constant selects
fixed/baseline/final control. All use the same MATLAB component classes.
The estimator has separate outputs for estimated state [x,y,vx,vy], RMS position
uncertainty, initialized flag, and RFID availability. The controller receives
all four; logging receives state/uncertainty/initialization and RFID measurements.
The model uses normal-mode Level-2 MATLAB S-functions, not generated code.
main loads the saved model, sets controlConfig/controlScenario in its model
workspace, and opens it for GUI Run. It never rebuilds, saves, or closes it. An already
open model keeps its GUI edits. Save diagram changes explicitly in Simulink.
The model's timing expressions use controlConfig.dt and controlConfig.stopTime.
Each subsystem instantiates its runtime component when simulation starts, so
every run starts with fresh sensor, filter, plant, controller and logger state.
For command-driven execution after main, use sim('rfid_tracking'). The batch
runner does this explicitly for each assignment case.
There is no model-generation script or second standalone simulation loop.

MODELS AND DEFAULTS
Tag state: x,y,vx,vy. Reader state: x,y,heading,velocity,turnSpeed.
Units: m, s, rad; commands are turn rates in rad/s (not turn angles).
Workspace: 10 x 10 m, no excluded regions. Each trajectory starts at t=0 and
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
starts at the next valid read. The returned comparison table reports compliance from actual
results. Unrecovered/interrupted episodes are not counted as successes.

SCOPE
One estimator only (PF), baseline/final controls, required scenario analyses,
metrics and static plots. No EKF, generic estimator framework, random walks,
particle-history export, HTML animation, JSON layer or software test suite.
Hardware/BOM, slides and actual team contributions remain manual deliverables.
Changes to the run flow have not been executed here because local MATLAB
startup/license checkout fails. No numerical results are fabricated.
