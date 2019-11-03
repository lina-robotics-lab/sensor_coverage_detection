close all;

distance_to_target = 1;
target_loc = [0,0];

num_sensors = 10;
initial_angles = 0.1*pi*rand(1,num_sensors);
% initial_angles = 0.1*[0,1,2];

sensors = SensorClass.empty(0,num_sensors);
for i=1:num_sensors
    angle = initial_angles(i);
    initial_loc = target_loc+distance_to_target*[cos(angle),sin(angle)];
    s = SensorClass(initial_loc,target_loc);
    sensors(i) = s;
end

for i = 1:100
   angles = zeros(1,num_sensors);
   % Step 1: measure target simultaneously, after which the angle state of
   % each sensor is automatically updated.
   for j=1:length(sensors)
       angles(j)=sensors(j).measureTarget(target_loc);
       % The angle of each sensor is automatically updated after calling measureTarget().
   end
   % Sort the angles in order to obtain cw(clockwise) and ccw(counter-clockwise).
   % The angles increase in counter-clockwise direction by default.
   [sorted_angles,sorted_indices] = sort(angles);
   % Sort is in ascending order by default.
   
   for j = 1:length(sorted_indices)
      curr_index = sorted_indices(j);
      cw_Neighbor = sorted_angles(cyclic_mod(j-1,num_sensors));
      ccw_Neighbor = sorted_angles(cyclic_mod(j+1,num_sensors));
      sensors(curr_index).moveSensor(cw_Neighbor, ccw_Neighbor); 
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

% Matlab use 1-based indexing, which makes cyclic array indexing by a[mod(i,p)]not
% directly available. But we can hack this in the following way.
function r=cyclic_mod(n,p)
% For example, cyclic_mod(1,3)=1,cyclic_mod(2,3)=2, cyclic_mod(3,3)=3,
% cyclic_mod(4,3)=1,cyclic_mod(5,3)=2...
    r = p-mod(-n,p);
end