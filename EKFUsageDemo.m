% This script demos how to use EKF to estimate the coordinate of a target 
% moving in 8-shaped trajectory in 2-D.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create the dynamics object
% Parameter for 8-shape movement.
% Sampling interval for target location.
dt = 0.01;
omega = 0.05*pi;
dynamics =  EightShapeDynamics(omega, dt);

% Create the measurement object
sensorLocs = [[0;1] [1;0] [-1;0] [0;-1]]; % By convention, locations should be an array of columns.
meas = Measurement();
meas.sensorLocs = sensorLocs;

% Create an ekf object
initial_location_estimation=[0.1;0.1];
ekf = extendedKalmanFilter(@dynamics.stateUpdate,@measure.measureUpdate,initial_location_estimation);
% Remark: dynamics.stateUpdate and measure.measureUpdate are both object methods

% Experiment 1: directly call ekf.predict() for a series of times, see what
% it produces.
space_dimension = size(sensorLocs);
max_iter= 1000;
predicts = zeros(space_dimension(1),max_iter);

actual_loc = [-0.1;-0.1];
actual_locs = zeros(space_dimension(1),max_iter);
for i=1:max_iter
    actual_loc=dynamics.stateUpdate(actual_loc);
    actual_locs(:,i)=actual_loc;
    predicts(:,i)=ekf.predict();
end
% plot(1:max_iter,predicts);
tiledlayout(2,1);
nexttile;
plot(predicts(1,:),predicts(2,:));
legend("Predicted Trajectory");
nexttile;
plot(actual_locs(1,:),actual_locs(2,:));
legend("Actual Trajectory");