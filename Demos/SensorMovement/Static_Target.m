% Script content: Move the sensors so that they distributed equi-angularly around a static
% target.

close all;

% The sensors always move around a circle with radius 1, the origin being
% the target.
sensor_dist_to_target = 1;

% The location of the static target.
target_loc = [0;0];

% Feel free to change the num_of_sensors and initial_angles here.
num_sensors = 3;
initial_angles = 0.1*pi*rand(1,num_sensors); 
k = 1/4; % Control gain for equi-angular control rule.
% k = 1/2;

% Initialization of sensors.
sensors = SensorClass.empty(0,num_sensors);
% Note: the sensors move along a boundary, which may not be a circled
% centered at the target location.
boundary_origin=[0.8;0];
% boundary_origin=target_loc;
% boundary_radius = 2;
boundary_radius = 0.5;

for i=1:num_sensors
    angle = initial_angles(i);
    initial_loc = boundary_origin+boundary_radius*[cos(angle);sin(angle)];
    s = SensorClass(initial_loc,boundary_origin,boundary_radius,k);
    sensors(i) = s;
end

% Entering main loop of the simulation
max_iteration = 100;
for i = 1:max_iteration
    move_sensors(sensors,target_loc);
end

%%%%%%%%%%%%%%Plot out sensor movement trajectories%%%%%%%%%%%%%%%%
marker_size=100;
scatter(target_loc(1),target_loc(2),marker_size,'h','filled','red','DisplayName','Target');
hold on;
plot_sensor_movement(sensors);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
