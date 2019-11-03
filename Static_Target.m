% Script content: Move the sensors so that they distributed equi-angularly around a static
% target.

close all;

% The sensors always move around a circle with radius 1, the origin being
% the target.
sensor_dist_to_target = 1;

% The location of the static target.
target_loc = [0,0];

% Feel free to change the num_of_sensors and initial_angles here.
num_sensors = 10;
initial_angles = 0.1*pi*rand(1,num_sensors); 
k = 1/4; % Control gain for equi-angular control rule.
% k = 1/2;

% Initialization of sensors.
sensors = SensorClass.empty(0,num_sensors);
for i=1:num_sensors
    angle = initial_angles(i);
    initial_loc = target_loc+sensor_dist_to_target*[cos(angle),sin(angle)];
    s = SensorClass(initial_loc,target_loc,k);
    sensors(i) = s;
end

% Entering main loop of the simulation
max_iteration = 100;
for i = 1:max_iteration
   angles = zeros(1,num_sensors);
   % Step 1: measure target simultaneously, after which the angle state of
   % each sensor is automatically updated.
   for j=1:length(sensors)
       angles(j)=sensors(j).measureTarget(target_loc);
       % The angle of each sensor is automatically updated after calling measureTarget().
   end
   
   % Step 2: Sort the angles in order to obtain the angles of cw_neighbor(clockwise) and ccw_neighor(counter-clockwise).
   % The angles increase in counter-clockwise direction by default.
   [sorted_angles,sorted_indices] = sort(angles);
   % Matlab sorting is in ascending order by default.
   
   % Step 3: move the sensors using predefined control rule.
   for j = 1:length(sorted_indices)
      curr_index = sorted_indices(j);
      cw_Neighbor = sorted_angles(cyclic_mod(j-1,num_sensors));
      ccw_Neighbor = sorted_angles(cyclic_mod(j+1,num_sensors));
      sensors(curr_index).moveSensor(cw_Neighbor, ccw_Neighbor,sensor_dist_to_target); 
   end
end

% out = [s1.returnAngle(), s2.returnAngle(), s3.returnAngle()];

%%%%%%%%%%%%%%Plot out sensor movement trajectories%%%%%%%%%%%%%%%%
marker_size=100;
scatter(target_loc(1),target_loc(2),marker_size,'h','filled','red','DisplayName','Target');
hold on;
for i=1:num_sensors
    s = sensors(i);
    scatter(s.states(1,1),s.states(1,2),marker_size,'d','DisplayName',"Sensor "+i+" Init Loc");
    hold on;
    plot(s.states(1:end,1),s.states(1:end,2),'DisplayName',"Sensor "+i+" Trajectory");
    hold on;
    scatter(s.states(end,1),s.states(end,2),'filled','DisplayName',"Sensor "+i+" Final Loc");
    hold on;
end
legend();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Matlab uses 1-based indexing, which makes cyclic array indexing by a[mod(i,p)]not
% directly available. But we can hack our way out as follows:
function r=cyclic_mod(n,p)
    r = p-mod(-n,p);
end
% The function behavior is the following: cyclic_mod(1,3)=1,cyclic_mod(2,3)=2, cyclic_mod(3,3)=3,
% cyclic_mod(4,3)=1,cyclic_mod(5,3)=2...
