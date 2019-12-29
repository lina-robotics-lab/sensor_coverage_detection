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

omega=0.1;
total_time=90;
max_iter=total_time/dt;




enable_sensor_movement=false;
[corrects_1,predicts_1,actual_locs,sensors,plant_measurements]=EKF_Robustness(@move_sensors_equi_angular);

plot(2:max_iter,plant_measurements);

plot_trajectories(predicts_1,actual_locs,sensors,b,enable_sensor_movement);
legend();
figure;
plot_error(predicts_1,actual_locs,total_time,b,enable_sensor_movement);
