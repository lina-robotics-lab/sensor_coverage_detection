% close all;
% This script demos how to use EKF to estimate the coordinate of a target 
% moving in 8-shaped trajectory in 2-D.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [predicts,actual_locs,sensors]=EKF_MovingSensor()
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

    initial_angles = pi+0.05*(0.5-rand(num_sensors)); 
%     initial_angles=2*pi/num_sensors * [1:num_sensors];

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

    % Create the measurement object
    % Assume the sensors are placed at fixed locations.
    % In the future,we can dynamically change meas.sensorLocs to represent the movement of
    % mobile sensors.

    mus = zeros(length(sensorLocs),1);
    measure_noise = measure_noise_variance*eye(length(sensorLocs));
    proc_noise = proc_noise_variance*eye(space_dimension);
    meas = Measurement(b,measure_noise_variance);
    meas.sensorLocs = sensorLocs;

    %  Create an ekf object
    ekf = extendedKalmanFilter(@dynamics.stateUpdate,@meas.measureUpdatePerfect,initial_location_estimation);
    ekf.ProcessNoise = proc_noise;
    ekf.MeasurementNoise = measure_noise;
    ekf.MeasurementJacobianFcn =  @meas.measureJacobian;
    % Remark: dynamics.stateUpdate and meas.measureUpdate are both object methods

    predicts = zeros(space_dimension,max_iter);
    actual_locs = zeros(space_dimension,max_iter);

    predicts(:,1)=initial_location_estimation;
    actual_loc=initial_target_loc;
    actual_locs(:,1)=actual_loc;

    for i = 2:max_iter
%         actual_loc=dynamics.stateUpdateWithNoise(actual_loc);
       actual_loc=dynamics.stateUpdate(actual_loc); % We use original state update here so that the actual trajectory does not cramp up.
       actual_locs(:,i)=actual_loc;

        % First, update the sensorLocs array in the measurement object
        for j=1:num_sensors
            sensorLocs(:, j) = sensors(j).returnPos();
        end
        meas.sensorLocs = sensorLocs;

        % Second, make the measurement, with noise added..
        plant_measurement = meas.measureUpdateWithNoise(actual_loc);

        % Third, make estimation of target location using ekf. Also update ekf.

        % Important: after calling ekf.correct(), we must also call
        % ekf.predict() for the ekf states to be properly updated! If we do not
        % call ekf.predict() after calling ekf.correct(), the state of ekf will
        % be wrong!
        ekf.correct(plant_measurement);
        estimated_loc=ekf.predict();
        predicts(:,i)=estimated_loc;


        %Fourth, move the sensors w.r.t estimated_loc
        if enable_sensor_movement
            move_sensors(sensors,estimated_loc);
        end

    end
        
end


