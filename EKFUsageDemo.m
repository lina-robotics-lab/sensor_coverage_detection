close all;
% This script demos how to use EKF to estimate the coordinate of a target 
% moving in 8-shaped trajectory in 2-D.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 0.1;
% dt = 1;

% omega = 100;
omega = 0.1;

total_time = 62;%Select total time carefully so that we do not encounters the crossing point. As that point will make state update unstable.
max_iter= floor(total_time/dt);

% Sensor Initialization
% Feel free to change the num_of_sensors and initial_angles here.
num_sensors = 3;
k = 1/4; % Control gain for equi-angular control rule.
% k = 1/2;
% Initialization of sensors.
sensors = SensorClass.empty(0,num_sensors);
% Note: the sensors move along a boundary, which may not be a circled
% centered at the target location.
% boundary_origin=[0.8;0];
boundary_origin=[0;0];
initial_angles = 0.1*pi*rand(1,num_sensors); 
boundary_radii = 1.5*ones(1,num_sensors);

sensor_locs = zeros(2, num_sensors);

for i=1:num_sensors
    angle = initial_angles(i);
    initial_loc = boundary_origin+boundary_radii(i)*[cos(angle);sin(angle)];
    s = SensorClass(initial_loc,boundary_origin,boundary_radii(i),k);
    sensors(i) = s;
    sensor_locs(:, i) = s.returnPos();
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
    predicts(:,i)=ekf.predict();
end

% tiledlayout(2,1);

% nexttile;

plot(predicts(1,:),predicts(2,:));
title('Predicted Trajectory-No correction');
% nexttile;
figure;
plot(actual_locs(1,:),actual_locs(2,:));
title("Actual Trajectory");

% Demo 2: continue the simulation, but call ekf.correct() to correct the state of ekf by letting
% it see the actual state.
actual_loc = [0.01;0.01]; 
% initial_location_estimation=actual_loc;
initial_location_estimation=[-1;0.2];

ekf = extendedKalmanFilter(@dynamics.stateUpdate,@meas.measureUpdate,initial_location_estimation);
ekf.ProcessNoise = proc_noise;
ekf.MeasurementNoise = measure_noise;
ekf.MeasurementJacobianFcn =  @meas.measureJacobian;

predicts = zeros(space_dimension,max_iter);
actual_locs = zeros(space_dimension,max_iter);

for i = 1:max_iter
    actual_loc=dynamics.stateUpdate(actual_loc);
    actual_locs(:,i)=actual_loc;
    
    plant_measurement = meas.measureUpdate(actual_loc);
    
    % Important: after calling ekf.correct(), we must also call
    % ekf.predict() for the ekf states to be properly updated! If we do not
    % call ekf.predict() after calling ekf.correct(), the state of ekf will
    % be wrong!
    ekf.correct(plant_measurement);
    predicts(:,i)=ekf.predict();
    
end

figure;
% tiledlayout(3,1);
% nexttile;
plot(predicts(1,:),predicts(2,:));
title('Predicted Trajectory-With Correction');
% nexttile;
figure;
plot(actual_locs(1,:),actual_locs(2,:));
title("Actual Trajectory");

% nexttile;
figure;
error=sum((predicts-actual_locs).^2,1);
plot(linspace(0,total_time,length(error)),error);
title("error");
% You should see after we incorporate ekf.correct(), the initially offed
% estimation can be gradually corrected. You should also see some noisy
% behavior if we tune up the noise magnitude in the beginning of this file.