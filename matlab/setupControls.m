function root = setupControls()
%SETUPCONTROLS Add only the MATLAB port and its Simulink adapters to the path.
root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root, 'simulink'));
end
