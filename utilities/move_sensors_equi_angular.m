function move_sensors_equi_angular(sensors,target_loc,plant_measurement)
   num_sensors =length(sensors);
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
      
      % Major difference to Static_Target: we need to calculate
      % sensor_dist_to_target dynamically.
      
      sensors(curr_index).moveSensorEquiAngular(cw_Neighbor, ccw_Neighbor,target_loc); 
   end
end
