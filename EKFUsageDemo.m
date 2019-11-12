%% Remeber to first set the simulation parameters in setEKFUsageDemoParams.m, then run this script.
close all;
setEKFUsageDemoParams(); % Assign the simulation parameters with default values.

% Then, manually tweak some of the parameters to meet the need of our
% current experiment.
global enable_sensor_movement;
global omega;

omega=1e-4; % The case when omega being small is a sanity check, that the target barely moves.
% omega=0.1;

enable_sensor_movement=true;
EKF_MovingSensor;
enable_sensor_movement=false;
EKF_MovingSensor;