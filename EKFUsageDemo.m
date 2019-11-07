close all;
% This script demos how to use EKF to estimate the coordinate of a target 
% moving in 8-shaped trajectory in 2-D.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global dt;
global omega;
global total_time;
global max_iter;
global num_sensors;
global k;
global boundary_origin;
global b;
global measure_noise_mag;
global proc_noise_mag;
global actual_loc; 
global initial_location_estimation;
setEKFUsageDemoParams();
% Initialization of sensors.
sensors = SensorClass.empty(0,num_sensors);
% Note: the sensors move along a boundary, which may not be a circled
% centered at the target location.
initial_angles = [0 0.3 0.2 0.1]; 
boundary_radii = 1.5*ones(1,num_sensors);
sensorLocs = zeros(2, num_sensors);

for i=1:num_sensors
    angle = initial_angles(i);
    initial_loc = boundary_origin+boundary_radii(i)*[cos(angle);sin(angle)];
    s = SensorClass(initial_loc,boundary_origin,boundary_radii(i),k);
    sensors(i) = s;
    sensorLocs(:, i) = s.returnPos();
end

% sensorLocs = [[0;1.5] [1.5;0] [-1.5;0] [0;-1.5]]; % By convention, locations should be an array of columns.
% % sensorLocs = [[0;1.5] [-1.5;0] ]; % By convention, locations should be an array of columns.
% 
space_dimension = size(sensorLocs);
space_dimension = space_dimension(1);

% Create the dynamics object
% Parameter for 8-shape movement.
% Sampling interval for target location.
dynamics =  EightShapeDynamics(omega, dt);

% Create the measurement object
% Assume the sensors are placed at fixed locations.
% In the future,we can dynamically change meas.sensorLocs to represent the movement of
% mobile sensors.

mus = zeros(length(sensorLocs),1);
measure_noise = 0.05*eye(length(sensorLocs));
proc_noise = 1e-5*eye(space_dimension);
% b = 1;
b = -2;
meas = Measurement(b);
meas.sensorLocs = sensorLocs;



% Demo 1: directly call ekf.predict() for a series of times, see what
% it produces.
actual_loc = [0.01;0.01]; 
% initial_location_estimation=actual_loc;
initial_location_estimation=[0;0.2];

%  Create an ekf object
ekf = extendedKalmanFilter(@dynamics.stateUpdate,@meas.measureUpdate,initial_location_estimation);
ekf.ProcessNoise = proc_noise;
ekf.MeasurementNoise = measure_noise;
ekf.MeasurementJacobianFcn =  @meas.measureJacobian;
% Remark: dynamics.stateUpdate and meas.measureUpdate are both object methods

predicts = zeros(space_dimension,max_iter);
actual_locs = zeros(space_dimension,max_iter);

for i=1:max_iter
    actual_loc=dynamics.stateUpdate(actual_loc);
    actual_locs(:,i)=actual_loc;
    
    % First, update the sensorLocs array in the measurement object
    for j=1:num_sensors
        sensorLocs(:, j) = sensors(j).returnPos();
    end
    meas.sensorLocs = sensorLocs;
    
    % Second, make the measurement.
    plant_measurement = meas.measureUpdate(actual_loc);
    
    % Third, make estimation of target location using ekf. Also update ekf.
    
    % Here we deliberately do not call correct function to update ekf.
    
    %ekf.correct(plant_measurement);
    estimated_loc=ekf.predict();
    predicts(:,i)=estimated_loc;
    
    % Fourth, feed the estimated_loc to the sensors, use control rule to
    % move the sensors.
    move_sensors(sensors,estimated_loc);
end

plot(predicts(1,:),predicts(2,:),'DisplayName','Predicted Trajectory');
hold on;
plot(actual_locs(1,:),actual_locs(2,:),'DisplayName','Actual Trajectory');
hold on;
plot_sensor_movement(sensors);
title('No correction');
legend();
