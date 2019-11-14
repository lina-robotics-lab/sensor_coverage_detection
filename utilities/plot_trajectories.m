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
