%% Remeber to first set the simulation parameters in setEKFUsageDemoParams.m, then run this script.
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

close all;
setEKFUsageDemoDefaultParams(); % Assign the simulation parameters with default values.

% Then, manually tweak some of the parameters to meet the need of our
% current experiment.

 % The case when omega being small is a sanity check, that the target barely moves.
omega=1e-4;
% omega=0.1;
b=1;
total_time = 100;%Select total time carefully so that we do not encounters the crossing point. As that point will make state update unstable.
dt = 0.3;
max_iter= floor(total_time/dt); 


enable_sensor_movement=true;
[predicts,actual_locs,sensors]=EKF_MovingSensor();

plot_trajectories(predicts,actual_locs,sensors,b,enable_sensor_movement);

figure;
plot_error(predicts,actual_locs,total_time,b,enable_sensor_movement);
hold on;


enable_sensor_movement=false;
[predicts,actual_locs,sensors]=EKF_MovingSensor();
plot_error(predicts,actual_locs,total_time,b,enable_sensor_movement);
legend();


plot_trajectories(predicts,actual_locs,sensors,b,enable_sensor_movement);


function plot_trajectories(predicts,actual_locs,sensors,b,enable_sensor_movement)
    figure;
    scatter(predicts(1,1),predicts(2,1),'d','DisplayName','Initial Predicted Loc');
    hold on;
    plot(predicts(1,:),predicts(2,:),'DisplayName','Predicted Trajectory');
    hold on;
    scatter(predicts(1,end),predicts(2,end),'+','DisplayName','Final Predicted Loc');
    hold on;

    plot(actual_locs(1,:),actual_locs(2,:),'DisplayName','Actual Trajectory');
    hold on;
    plot_sensor_movement(sensors);
    title("Trajectories, b="+b+", Moving Sensors:"+enable_sensor_movement);
end

function plot_error(predicts,actual_locs,total_time,b,enable_sensor_movement)
   
    error=sum((predicts-actual_locs).^2,1);
    plot(linspace(0,total_time,length(error)),error,'DisplayName',"Moving Sensors:"+enable_sensor_movement);
    title("Estimation Error. b="+b);
    xlabel("Time:seconds")
    ylabel("Error")
end