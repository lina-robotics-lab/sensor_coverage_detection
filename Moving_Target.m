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
omega = 10;
total_time = 62;%Select total time carefully so that we do not encounters the crossing point. As that point will make state update unstable.
max_iter= floor(total_time/dt);

dynamics =  EightShapeDynamics(omega, dt);% Create the dynamics object


% Feel free to change the num_of_sensors and initial_angles here.
num_sensors = 3;
initial_angles = 0.1*pi*rand(1,num_sensors); 
initial_radii = 1.5*ones(1,num_sensors);
origin=[0,0];
k = 1/4; % Control gain for equi-angular control rule.
% k = 1/2;

% Initialization of sensors.
sensors = SensorClass.empty(0,num_sensors);
for i=1:num_sensors
    angle = initial_angles(i);
    initial_loc = origin+initial_radii(i)*[cos(angle),sin(angle)];
    s = SensorClass(initial_loc,target_loc,k);
    sensors(i) = s;
end

% Entering main loop of the simulation
target_locs = zeros(max_iter,space_dimension);
for i = 1:max_iter
   angles = zeros(1,num_sensors);
   % Step 0: Update Target Location
   target_locs(i,:)=target_loc';
   target_loc=dynamics.stateUpdate(target_loc);
  
   
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
      
      % Major difference to Static_Target: we need to calculate
      % sensor_dist_to_target dynamically.
      
      sensors(curr_index).moveSensor(cw_Neighbor, ccw_Neighbor,dist_to_target); 
   end
end

% out = [s1.returnAngle(), s2.returnAngle(), s3.returnAngle()];

%%%%%%%%%%%%%%Plot out sensor movement trajectories%%%%%%%%%%%%%%%%
marker_size=100;
plot(target_locs(1:end,1),target_locs(1:end,2),'DisplayName',"Target "+" Trajectory");
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
