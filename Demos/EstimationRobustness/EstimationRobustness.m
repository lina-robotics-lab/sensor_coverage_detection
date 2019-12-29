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
global enable_sensor_movement;


close all;
setEKFUsageDemoDefaultParams(); % Assign the simulation parameters with default values.

omega=0.1;
total_time=90;
max_iter=total_time/dt;


% Initialize two matlab struct for measurement parameters and corresponding
% 
params.K=1; params.R1=1e5; params.R0=1e-5; params.c1=0; params.c2=0; params.b=b;
param_errs.dK=0; param_errs.dR1=0; param_errs.dR0=0; param_errs.dc1=0; param_errs.dc2=0; param_errs.db=0;

enable_sensor_movement=true;

s="";
fields=fieldnames(params);
for f=1:length(fields)
    s=s+fields{f}+"="+getfield(params,fields{f})+" ";
end


linestyle={'-',':','-.','--'};
markertype = '.ox+*sdv^<>ph';

%% Run a benchmark simulation with no parameter errors. Plot the trajectories.
param_errs.dK=0; param_errs.dR1=0; param_errs.dR0=0; param_errs.dc1=0; param_errs.dc2=0; param_errs.db=0;
param_errs.dc1=0.5;
enable_sensor_movement=true;
[corrects,predicts,actual_locs,sensors,plant_measurements]=EKF_Robustness(@move_sensors_equi_angular,params,param_errs);
plot_trajectories(predicts,actual_locs,sensors,b,enable_sensor_movement);
title({"Default Parameters:"+s," Moving Sensors:"+enable_sensor_movement})
legend()

%% Error in K

% Reset parameter errors.
param_errs.dK=0; param_errs.dR1=0; param_errs.dR0=0; param_errs.dc1=0; param_errs.dc2=0; param_errs.db=0;

lind=1;
mind=1;
figure;
for dK=[-0.5,-0.1,0,0.1,0.5,1,2]
    param_errs.dK=dK;
    [corrects_1,predicts_1,actual_locs,sensors,plant_measurements]=EKF_Robustness(@move_sensors_equi_angular,params,param_errs);
    p=plot_error_robustness(predicts_1,actual_locs,total_time, params);
    set(p,"DisplayName","dK="+param_errs.dK,"linestyle",linestyle{lind},"marker",markertype(mind));
    lind=int8(mod(lind,length(linestyle))+1);
    mind=mod(mind+floor(lind/length(linestyle)),length(markertype));
    hold on;
end
title({"Sensitivity to K","Default Parameters:"+s," Moving Sensors:"+enable_sensor_movement})
xlabel("Time:seconds")
ylabel("Error")
legend()

%% Error in b, with b+db having the same sign as b
% Reset parameter errors.
param_errs.dK=0; param_errs.dR1=0; param_errs.dR0=0; param_errs.dc1=0; param_errs.dc2=0; param_errs.db=0;

lind=1;
mind=1;
figure;
for db=[-2,-1,-0.5,-0.1,0,0.1,0.5,1,1.5]
    param_errs.db=db;
    [corrects_1,predicts_1,actual_locs,sensors,plant_measurements]=EKF_Robustness(@move_sensors_equi_angular,params,param_errs);
    p=plot_error_robustness(predicts_1,actual_locs,total_time, params);
    set(p,"DisplayName","db="+param_errs.db,"linestyle",linestyle{lind},"marker",markertype(mind));
    lind=int8(mod(lind,length(linestyle))+1);
    mind=mod(mind+floor(lind/length(linestyle)),length(markertype));
    hold on;
end
title({"Sensitivity to b","Default Parameters:"+s," Moving Sensors:"+enable_sensor_movement})
xlabel("Time:seconds")
ylabel("Error")
legend()

%% Error in b, with b+db having a different sign as b
lind=1;
mind=1;
figure;
for db=[-b+0.5,-b+1]
    param_errs.db=db;
    [corrects_1,predicts_1,actual_locs,sensors,plant_measurements]=EKF_Robustness(@move_sensors_equi_angular,params,param_errs);
    p=plot_error_robustness(predicts_1,actual_locs,total_time, params);
    set(p,"DisplayName","db="+param_errs.db,"linestyle",linestyle{lind},"marker",markertype(mind));
    lind=int8(mod(lind,length(linestyle))+1);
    mind=mod(mind+floor(lind/length(linestyle)),length(markertype));
    hold on;
end
title({"Sensitivity to b","Default Parameters:"+s," Moving Sensors:"+enable_sensor_movement})
xlabel("Time:seconds")
ylabel("Error")
legend()

%% Error in c1
% Reset parameter errors.
param_errs.dK=0; param_errs.dR1=0; param_errs.dR0=0; param_errs.dc1=0; param_errs.dc2=0; param_errs.db=0;

lind=1;
mind=1;
figure;
for dc1=[0,0.1,0.2,0.3,0.4,0.48]
    param_errs.dc1=dc1;
    [corrects_1,predicts_1,actual_locs,sensors,plant_measurements]=EKF_Robustness(@move_sensors_equi_angular,params,param_errs);
    p=plot_error_robustness(predicts_1,actual_locs,total_time, params);
    set(p,"DisplayName","dc1="+param_errs.dc1,"linestyle",linestyle{lind},"marker",markertype(mind));
    lind=int8(mod(lind,length(linestyle))+1);
    mind=mod(mind+floor(lind/length(linestyle)),length(markertype));
    hold on;
end
title({"Sensitivity to c1","Default Parameters:"+s," Moving Sensors:"+enable_sensor_movement})
xlabel("Time:seconds")
ylabel("Error")
legend()

%% Error in c2
% Reset parameter errors.
param_errs.dK=0; param_errs.dR1=0; param_errs.dR0=0; param_errs.dc1=0; param_errs.dc2=0; param_errs.db=0;

lind=1;
mind=1;
figure;
for dc2=[0,0.1,0.5,1,2]
    param_errs.dc2=dc2;
    [corrects_1,predicts_1,actual_locs,sensors,plant_measurements]=EKF_Robustness(@move_sensors_equi_angular,params,param_errs);
    p=plot_error_robustness(predicts_1,actual_locs,total_time, params);
    set(p,"DisplayName","dc2="+param_errs.dc2,"linestyle",linestyle{lind},"marker",markertype(mind));
    lind=int8(mod(lind,length(linestyle))+1);
    mind=mod(mind+floor(lind/length(linestyle)),length(markertype));
    hold on;
end
title({"Sensitivity to c2","Default Parameters:"+s," Moving Sensors:"+enable_sensor_movement})
xlabel("Time:seconds")
ylabel("Error")
legend()

function p=plot_error_robustness(predicts,actual_locs,total_time,params)
    error=sum((predicts-actual_locs).^2,1);
    p=plot(linspace(0,total_time,length(error)),error);
end
