
function plot_sensor_movement(sensors)
    marker_size=100;
    num_sensors = length(sensors);
    for i=1:num_sensors
        s = sensors(i);
        scatter(s.states(1,1),s.states(1,2),marker_size,'d','DisplayName',"Sensor "+i+" Init Loc");
        hold on;
        plot(s.states(1:end,1),s.states(1:end,2),'DisplayName',"Sensor "+i+" Trajectory");
        hold on;
        scatter(s.states(end,1),s.states(end,2),'filled','DisplayName',"Sensor "+i+" Final Loc");
        hold on;
    end
end