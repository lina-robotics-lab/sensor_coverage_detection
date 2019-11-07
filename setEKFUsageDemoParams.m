function setEKFUsageDemoParams()
    global dt;
    global omega;
    global total_time;
    global max_iter;
    global num_sensors;
    global k;
    global boundary_origin;
    global b;
    global measure_noise_mag;
    global proc_noise_mag;
    global actual_loc; 
    global initial_location_estimation;
    dt = 0.1;
%     dt = 1;

    % omega = 100;
    omega = 0.1;

    total_time = 62;%Select total time carefully so that we do not encounters the crossing point. As that point will make state update unstable.
    max_iter= floor(total_time/dt);

    % Sensor Initialization
    % Feel free to change the num_of_sensors and initial_angles here.
    num_sensors = 4;
    k = 1/4; % Control gain for equi-angular control rule.
    % k = 1/2;
    % boundary_origin=[0.8;0];
    boundary_origin=[0;0];
    b = 1;
%     b = -2;
    measure_noise_mag = 1.5;
    proc_noise_mag = 1e-5;

    actual_loc = [0.01;0.01]; 
%     initial_location_estimation=actual_loc;
    initial_location_estimation=[0;1];
end