function [modelFile,config]=buildSimulinkModel(scenarioFile,varargin)
%BUILDSIMULINKMODEL Build the six diagram subsystems using the MATLAB classes.
% Requires Simulink. Normal simulation only (interpreted Level-2 S-functions).
root=setupControls(); if nargin<1, scenarioFile=fullfile(root,'scenarios','assignment','straight.csv'); end
[config,scenario]=initControls(scenarioFile,varargin{:});
assert(license('test','Simulink'),'controls:Toolbox','Simulink is required to build the model.');
model='rfid_tracking';
assert(~bdIsLoaded(model),'controls:ModelOpen','Close rfid_tracking before rebuilding; unsaved edits are never discarded.');
new_system(model); cleanup=onCleanup(@()close_system(model,0));
set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','controlConfig.dt',...
    'StopTime','controlConfig.stopTime','SimulationMode','normal','ReturnWorkspaceOutputs','on',...
    'AlgebraicLoopMsg','error');
workspace=get_param(model,'ModelWorkspace'); assignin(workspace,'controlConfig',config); assignin(workspace,'controlScenario',scenario);
makeSubsystem(model,'Scenario generator','scenario',[],4,[40 70 200 135],'controlScenario','[]');
makeSubsystem(model,'RFID measurement','sensor',[4 5],6,[270 60 440 145],'controlConfig','[]');
makeSubsystem(model,'Tag position estimator','estimator',[6 5],7,[510 60 700 165],'controlConfig','[]');
makeSubsystem(model,'Feedback controller','controller',[7 5 1],2,[770 60 950 165],'controlConfig','[]');
makeSubsystem(model,'Reader-platform plant','plant',2,5,[780 290 950 365],'controlConfig','[]');
makeSubsystem(model,'Data logging and visualization','logger',[4 7 5 6 2],[],[400 310 640 490],...
    'controlConfig','controlScenario.events');
selector=[model '/Controller mode (0 fixed, 1 baseline, 2 final)'];
add_block('simulink/Sources/Constant',selector,'Value','controlConfig.controllerMode','Position',[760 210 940 245]);
connect(model,'Scenario generator/1','RFID measurement/1');
connect(model,'Reader-platform plant/1','RFID measurement/2');
connect(model,'RFID measurement/1','Tag position estimator/1');
connect(model,'Reader-platform plant/1','Tag position estimator/2');
connect(model,'Tag position estimator/1','Feedback controller/1');
connect(model,'Reader-platform plant/1','Feedback controller/2');
connect(model,'Controller mode (0 fixed, 1 baseline, 2 final)/1','Feedback controller/3');
connect(model,'Feedback controller/1','Reader-platform plant/1');
connect(model,'Scenario generator/1','Data logging and visualization/1');
connect(model,'Tag position estimator/1','Data logging and visualization/2');
connect(model,'Reader-platform plant/1','Data logging and visualization/3');
connect(model,'RFID measurement/1','Data logging and visualization/4');
connect(model,'Feedback controller/1','Data logging and visualization/5');
annotation=Simulink.Annotation(model,...
    ['CSV truth -> sensor + offline logger only. Estimator/controller never receive true tag state.' newline ...
    'Reader plant has no direct feedthrough: Outputs gives state(t); Update applies command for t+dt.' newline ...
    'Workspace: [0,10] x [0,10] m. Selector: 0 fixed, 1 baseline, 2 quality-aware standoff.']);
annotation.Position=[40 550 960 620];
folder=fullfile(root,'models'); if ~isfolder(folder), mkdir(folder); end
modelFile=fullfile(folder,[model '.slx']); save_system(model,modelFile);
clear cleanup; fprintf('Built %s\n',modelFile);
end

function makeSubsystem(model,name,role,inputWidths,outputWidths,position,settings,events)
path=[model '/' name]; add_block('built-in/Subsystem',path,'Position',position);
adapter=[path '/MATLAB implementation'];
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',adapter,...
    'FunctionName','trackingBlock','Parameters',sprintf('''%s'',''%s'',''%s''',role,settings,events),...
    'Position',[130 40 340 max(110,35+35*max(numel(inputWidths),numel(outputWidths)))]);
for i=1:numel(inputWidths)
    port=[path sprintf('/Input%d',i)]; add_block('built-in/Inport',port,'Port',num2str(i),...
        'PortDimensions',num2str(inputWidths(i)),'Position',[20 30+35*i 50 44+35*i]);
    add_line(path,sprintf('Input%d/1',i),sprintf('MATLAB implementation/%d',i),'autorouting','on');
end
for i=1:numel(outputWidths)
    port=[path sprintf('/Output%d',i)]; add_block('built-in/Outport',port,'Port',num2str(i),...
        'Position',[410 30+35*i 440 44+35*i]);
    add_line(path,sprintf('MATLAB implementation/%d',i),sprintf('Output%d/1',i),'autorouting','on');
end
end

function connect(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
