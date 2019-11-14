%% Remeber to first set the simulation parameters in setEKFUsageDemoParams.m, then run this script.
global dt;
global omega;
global total_time;
global max_iter;
global num_sensors;
global k;
global boundary_origin;
global b;
global measure_noise_variance;
global proc_noise_variance;
global initial_target_loc; 
global initial_location_estimation;
global enable_sensor_movement;
close all;
setEKFUsageDemoDefaultParams(); % Assign the simulation parameters with default values.

% Then, manually tweak some of the parameters to meet the need of our
% current experiment.

 % The case when omega being small is a sanity check, that the target barely moves.
omega=1e-1;
% omega=0.1;
b=-2;
total_time = 60;%Select total time carefully so that we do not encounters the crossing point. As that point will make state update unstable.
dt = 1;
max_iter= floor(total_time/dt); 
num_sensors=2;

enable_sensor_movement=true;
[predicts,actual_locs,sensors]=EKF_MovingSensor(@move_sensors_gradient);

plot_trajectories(predicts,actual_locs,sensors,b,enable_sensor_movement);

figure;
plot_error(predicts,actual_locs,total_time,b,enable_sensor_movement);
hold on;


enable_sensor_movement=false;
[predicts,actual_locs,sensors]=EKF_MovingSensor(@move_sensors_gradient);
plot_error(predicts,actual_locs,total_time,b,enable_sensor_movement);
legend();


plot_trajectories(predicts,actual_locs,sensors,b,enable_sensor_movement);

