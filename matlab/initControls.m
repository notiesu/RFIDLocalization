function [config,scenario]=initControls(scenarioFile,varargin)
% Configuration shared by all six Simulink subsystems.
root=setupControls();
if nargin<1, scenarioFile=fullfile(root,'scenarios','assignment','straight.csv'); end
p=inputParser; addParameter(p,'particles',500); addParameter(p,'seed',42);
addParameter(p,'sensorConfig',controls.RFIDConfig()); addParameter(p,'controllerConfig',controls.ControllerConfig());
addParameter(p,'controllerMode',1); addParameter(p,'output',fullfile(root,'output','tracking.csv'));
addParameter(p,'plot',true); parse(p,varargin{:}); config=p.Results;
scenario=controls.ScenarioGenerator.loadCSV(scenarioFile); config.dt=scenario.dt;
config.stopTime=(numel(scenario.states)-1)*scenario.dt; config.workspace=[0 10 0 10];
scenario.workspace=config.workspace;
config.sensorConfig.validate();
assert(ismember(config.controllerMode,[0 1 2]),'controls:Config','Controller mode must be 0, 1 or 2.');
assert(config.sensorConfig.updateRate*config.dt<=1+1e-12,'controls:Timing','dt must be <=1/updateRate.');
assert(isscalar(config.particles)&&config.particles>=2&&fix(config.particles)==config.particles,'controls:Config','Invalid particle count.');
assert(~isempty(config.output),'controls:Config','Specify an output CSV.');
[exists,input]=fileattrib(scenarioFile); [outputExists,output]=fileattrib(config.output);
assert(exists&&(~outputExists||~strcmp(input.Name,output.Name)),'controls:Path','Output cannot overwrite the input CSV.');
% Provisional design targets, not assignment-specified numerical thresholds.
config.requirements=struct('positionRMSE',0.5,'validTimeRate',95,'recoveryTime',3);
end
