function plot_error(predicts,actual_locs,total_time,b,enable_sensor_movement)
   
    error=sum((predicts-actual_locs).^2,1);
    plot(linspace(0,total_time,length(error)),error,'DisplayName',"Moving Sensors:"+enable_sensor_movement);
    title("Estimation Error. b="+b);
    xlabel("Time:seconds")
    ylabel("Error")
end
