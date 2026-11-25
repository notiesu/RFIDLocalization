function [logger,config]=main(scenarioFile,varargin)
% Run the assignment Simulink model; no duplicate MATLAB-only simulation loop.
[modelFile,config]=buildSimulinkModel(scenarioFile,varargin{:});
load_system(modelFile); cleanup=onCleanup(@()close_system('rfid_tracking',0));
sim('rfid_tracking'); logger=evalin('base','trackingLog');
metrics=logger.stats(); entries=logger.entries;
[folder,name,~]=fileparts(config.output); save(fullfile(folder,[name '.mat']),'config','metrics','entries');
logger.printStats();
if config.plot, plotResults(logger,fullfile(folder,[name '.png'])); end
end
