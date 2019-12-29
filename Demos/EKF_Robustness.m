% close all;
% This is the code base for studying the robustness of EKF estimation under
% imperfect knowledge of measurement function. The simulation code largely
% inherit from EKF_MovingSensor.m, except the use of measurement
% object is different.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [corrects,predicts,actual_locs,sensors,plant_measurements]=EKF_Robustness(move_sensor_function,params,param_errs)
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

%     dynamics =  EightShapeDynamics(omega, dt);
%     dynamics =  Revised_EightShapeDynamics(omega, dt);
%     dynamics = Cheating_EightShapeDynamics(omega,dt);
%     dynamics = StraightShapeDynamics(omega, dt);
    dynamics = CircleShapeDynamics(omega, dt);
%     dynamics=StationaryDynamics(omega,dt);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Important: Create two measurement objects, ideal_meas and actual_meas, with slightly different model
%%% parameters. We will eventually feed ideal_meas to the EKF estimator and use actual_meas
%%% to generate the actual measurement data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    mus = zeros(length(sensorLocs),1);
    measure_noise = measure_noise_variance*eye(length(sensorLocs));
    proc_noise = proc_noise_variance*eye(length(initial_target_loc));
    
    
    c1=params.c1; c2=params.c2; R1=params.R1; R0=params.R0; K=params.K;
    dc1=param_errs.dc1; dc2=param_errs.dc2; dR1=param_errs.dR1; dR0=param_errs.dR0; dK=param_errs.dK;
    db=param_errs.db;
    
    ideal_meas = Measurement(b,measure_noise_variance,c1,c2,R1,R0,K);    
    actual_meas = Measurement(b+db,measure_noise_variance,c1+dc1,c2+dc2,R1+dR1,R0+dR0,K+dK);
    ideal_meas.sensorLocs = sensorLocs;
    actual_meas.sensorLocs = sensorLocs;

    %  Create an ekf object. Notice we feed ideal_meas to the ekf
    %  estimator.
    ekf = extendedKalmanFilter(@dynamics.stateUpdate,@ideal_meas.measureUpdatePerfect,initial_location_estimation);
    ekf.ProcessNoise = proc_noise;
    ekf.MeasurementNoise = measure_noise;
    % Remark: dynamics.stateUpdate and meas.measureUpdate are both object methods
    
    % Declare book-keeping data structures
    predicts = zeros(space_dimension,max_iter);
    corrects = zeros(space_dimension,max_iter);

    actual_loc=initial_target_loc;
    actual_locs = zeros(space_dimension,max_iter);
    actual_locs(:,1)=actual_loc;
    
    plant_measurements=[];
    predicts(:,1)=ekf.State(end-1:end);
    
    % Enter the main loop of simulation
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
        
        % We need to update sensor locs for both the measurement object.
        ideal_meas.sensorLocs(:,:) = sensorLocs(:,:);
        actual_meas.sensorLocs(:,:) = sensorLocs(:,:);

        % Second, make the measurement, with noise added. Notice we use
        % actual_meas to generate the measurement data.
        plant_measurement = actual_meas.measureUpdateWithNoise(actual_loc);
        plant_measurements = [plant_measurements,plant_measurement];
        
        % Correct the current predicted state based on observation.       
        ekf.correct(plant_measurement);
        corrects(:,i)=ekf.State(end-1:end);
     
        %Fourth, move the sensors w.r.t estimated_loc
        if enable_sensor_movement
            move_sensor_function(sensors,ekf.State(end-1:end),plant_measurement);
        end
        
    end
end


