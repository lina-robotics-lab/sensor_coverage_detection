function setEKFUsageDemoDefaultParams()
    global dt;
    global omega;
    global total_time;
    global max_iter;
    global num_sensors;
    global k;
    global boundary_origin;
    global b;
    global measure_noise_variance;
    global proc_noise_variance;
    global initial_target_loc; 
    global initial_location_estimation;
    global descent_step_size;
    global Min_Separation;
    global mu;
    global barrier_order;
   
    descent_step_size=0.1;
    dt = 1;
 
    omega = 0.0;

    total_time = 60;%Select total time carefully so that we do not encounters the crossing point. As that point will make state update unstable.
    max_iter= floor(total_time/dt);

    % Sensor Initialization
    % Feel free to change the num_of_sensors and initial_angles here.
    num_sensors = 4;
    k = 1/4; % Control gain for equi-angular control rule.
    boundary_origin=[0;0];
    b = -2;
    measure_noise_variance = 5e-2;
    proc_noise_variance = 1e-5;
   
    a0=0.0;
%     initial_target_loc = [sin(a0);sin(a0)*cos(a0)]; 
    initial_target_loc = [0.5 * sin(a0); 0.5 * cos(a0)];
    initial_location_estimation=initial_target_loc;
    
    % Parameters for the barrier function.
    Min_Separation=2;
    mu=0.00000;
    barrier_order=5;
end