function config=main(scenarioFile,varargin)
% Initialize the saved model. Use Simulink Run to execute the simulation.
root=setupControls();
if nargin<1, scenarioFile=generateAssignmentScenario('straight'); end
[config,scenario]=initControls(scenarioFile,varargin{:});
modelFile=fullfile(root,'models','rfid_tracking.slx');
assert(isfile(modelFile),'controls:ModelMissing','Missing model: %s. Restore the saved SLX file.',modelFile);
% load_system preserves the model if it is already open, including GUI edits.
load_system(modelFile);
workspace=get_param('rfid_tracking','ModelWorkspace');
assignin(workspace,'controlConfig',config);
assignin(workspace,'controlScenario',scenario);
open_system('rfid_tracking');
end
