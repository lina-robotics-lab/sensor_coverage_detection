% Script Content: the goal is similar to Static_Target, the difference is the
% target now is moving(with EightShapeDynamics). No EKF is applied here, we
% assume the measurement is perfect.

close all;

% The initial location of the target.
target_loc = [0.001; 0.001];
space_dimension = size(target_loc);
space_dimension = space_dimension(1);


% Parameters for EightShapeDynamics.
dt = 0.1;
omega = 0.01;
total_time = 60;%Select total time carefully so that we do not encounters the crossing point. As that point will make state update unstable.
max_iter= floor(total_time/dt);

dynamics =  EightShapeDynamics(omega, dt);% Create the dynamics object


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

for i=1:num_sensors
    angle = initial_angles(i);
    initial_loc = boundary_origin+boundary_radii(i)*[cos(angle);sin(angle)];
    s = SensorClass(initial_loc,boundary_origin,boundary_radii(i),k);
    sensors(i) = s;
end


% Entering main loop of the simulation
target_locs = zeros(max_iter,space_dimension);
for i = 1:max_iter
   angles = zeros(1,num_sensors);
   % Step 0: Update Target Location
   target_locs(i,:)=target_loc';
   target_loc=dynamics.stateUpdate(target_loc);
    
   move_sensors(sensors,target_loc);
end

%%%%%%%%%%%%%%Plot out sensor movement trajectories%%%%%%%%%%%%%%%%
marker_size=100;
plot(target_locs(1:end,1),target_locs(1:end,2),'DisplayName',"Target "+" Trajectory");
hold on;
plot_sensor_movement(sensors);