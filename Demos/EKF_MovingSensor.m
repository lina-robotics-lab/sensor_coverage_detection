% close all;
% This script demos how to use EKF to estimate the coordinate of a target 
% moving in 8-shaped trajectory in 2-D.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [corrects,predicts,actual_locs,sensors,plant_measurements]=EKF_MovingSensor(move_sensor_function)
    global dt;
    global omega;
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

       % Initialization of sensors.
    sensors = SensorClass.empty(0,num_sensors);
    % Note: the sensors move along a boundary, which may not be a circled
    % centered at the target location.

%     initial_angles = pi+0.05*(0.5-rand(num_sensors)); 
    initial_angles=2*pi/num_sensors * [1:num_sensors];

    boundary_radii = 1.5*ones(1,num_sensors);
    sensorLocs = zeros(2, num_sensors);

    for i=1:num_sensors
        angle = initial_angles(i);
        initial_loc = boundary_origin+boundary_radii(i)*[cos(angle);sin(angle)];
        s = SensorClass(initial_loc,boundary_origin,boundary_radii(i),k);
        sensors(i) = s;
        sensorLocs(:, i) = s.returnPos();
    end

    space_dimension = size(sensorLocs);
    space_dimension = space_dimension(1);

    % Create the dynamics object
    % Parameter for 8-shape movement.
    % Sampling interval for target location.
    dynamics =  EightShapeDynamics(omega, dt);
%     dynamics =  Revised_EightShapeDynamics(omega, dt);
%     dynamics = Cheating_EightShapeDynamics(omega,dt);
    
    % Create the measurement object
    % Assume the sensors are placed at fixed locations.
    % In the future,we can dynamically change meas.sensorLocs to represent the movement of
    % mobile sensors.

    mus = zeros(length(sensorLocs),1);
    measure_noise = measure_noise_variance*eye(length(sensorLocs));
    proc_noise = proc_noise_variance*eye(length(initial_target_loc));
    meas = Measurement(b,measure_noise_variance);
    meas.sensorLocs = sensorLocs;

    %  Create an ekf object
    ekf = extendedKalmanFilter(@dynamics.stateUpdate,@meas.measureUpdatePerfect,initial_location_estimation);
    ekf.ProcessNoise = proc_noise;
    ekf.MeasurementNoise = measure_noise;
%     ekf.MeasurementJacobianFcn =  @meas.measureJacobian;
    % Remark: dynamics.stateUpdate and meas.measureUpdate are both object methods

    predicts = zeros(space_dimension,max_iter);
    corrects = zeros(space_dimension,max_iter);
    residuals = zeros(num_sensors,max_iter);
 
    actual_loc=initial_target_loc;
    actual_locs = zeros(space_dimension,max_iter);

    plant_measurements=[];
    predicts(:,1)=ekf.State(end-1:end);
    for i = 2:max_iter
        % Make a prediction about incoming state based on current corrected
        % ekf state.
        ekf.predict();
        predicts(:,i)=ekf.State(end-1:end);

        actual_loc=dynamics.stateUpdate(actual_loc); % We use original state update here so that the actual trajectory does not cramp up.
        actual_locs(:,i)=actual_loc(end-1:end);
     
        % First, update the sensorLocs array in the measurement object
        for j=1:num_sensors
            sensorLocs(:, j) = sensors(j).returnPos();
        end
        meas.sensorLocs(:,:) = sensorLocs(:,:);

        % Second, make the measurement, with noise added..
        plant_measurement = meas.measureUpdateWithNoise(actual_loc);
        plant_measurements = [plant_measurements,plant_measurement];
        
        % Correct the current predicted state based on observation.
        residual = ekf.residual(plant_measurement); % Book keeping step, this does not affect the state of ekf.
        
        ekf.correct(plant_measurement);
        corrects(:,i)=ekf.State(end-1:end);
     

        %Fourth, move the sensors w.r.t estimated_loc
        if enable_sensor_movement
            move_sensor_function(sensors,ekf.State(end-1:end),plant_measurement);
        end
        
    end
end


