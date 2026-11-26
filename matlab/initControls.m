function [config,scenario]=initControls(scenarioFile,varargin)
% Configuration shared by all six Simulink subsystems.
setupControls();
if nargin<1, scenarioFile=generateAssignmentScenario('straight'); end
p=inputParser; addParameter(p,'particles',500); addParameter(p,'seed',42);
addParameter(p,'sensorConfig',controls.RFIDConfig()); addParameter(p,'controllerConfig',controls.ControllerConfig());
addParameter(p,'controllerMode',1); parse(p,varargin{:}); config=p.Results;
if istable(scenarioFile)
    scenario=controls.ScenarioGenerator.fromTable(scenarioFile);
else
    scenario=controls.ScenarioGenerator.loadCSV(scenarioFile);
end
config.dt=scenario.dt;
config.stopTime=(numel(scenario.states)-1)*scenario.dt; config.workspace=[0 10 0 10];
scenario.workspace=config.workspace;
config.sensorConfig.validate();
assert(ismember(config.controllerMode,[0 1 2]),'controls:Config','Controller mode must be 0, 1 or 2.');
assert(config.sensorConfig.updateRate*config.dt<=1+1e-12,'controls:Timing','dt must be <=1/updateRate.');
assert(isscalar(config.particles)&&config.particles>=2&&fix(config.particles)==config.particles,'controls:Config','Invalid particle count.');
% Provisional design targets, not assignment-specified numerical thresholds.
config.requirements=struct('positionRMSE',0.5,'validTimeRate',95,'recoveryTime',3);
end
