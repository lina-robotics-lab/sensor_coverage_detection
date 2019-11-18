function move_sensors_gradient(sensors,target_loc,plant_measurement)
   global descent_step_size;
   num_sensors =length(sensors);
   p=zeros(2,num_sensors);
   h=plant_measurement;
   for i=1:num_sensors
       p(:,i)=sensors(i).returnPos()-target_loc;
   end
   % Calculate the directional vectors of each sensor
   r=p.^2;
   
   r=sqrt(r(1,:)+r(2,:));
   r_hat = p./r;
   
   t_hat=zeros(size(r_hat));
   t_hat(1,:)=-r_hat(2,:);
   t_hat(2,:)=r_hat(1,:);
   
   % Calculate the gradiens and move the sensor.
   for k=1:num_sensors
      [dLdeta,dLdr]= partial_derivatives(h,r_hat,k,num_sensors);
      gradient=dLdr*r_hat(:,k)+(dLdeta/r(k))*t_hat(:,k);
      sensors(k).moveSensorGradient(gradient,descent_step_size); 
   end
   
    function [dLdeta,dLdr]=partial_derivatives(h,r_hat,k,num_sensors)
        global b;
        global measure_noise_variance;
        C1 = (b^4/measure_noise_variance)*h(k)^(2-2/b);
           
        C2 = (b^4*(b-1)/measure_noise_variance)*h(k)^(2-3/b);
     
     
        sum1=0;
        sum2 = 0;
        for j=1:num_sensors
            rkrj=min(r_hat(:,k)'*r_hat(:,j),1);
                    direction = sign(det([r_hat(:,j),r_hat(:,k)]));
%            direction = sign(det([r_hat(:,j),r_hat(:,k)]));
        
            sum1=sum1+h(j)^(2-2/b)*rkrj*sqrt(1-rkrj^2)*direction;
            sum2=sum2+h(j)^(2-2/b)*(1-rkrj^2);
        end
        dLdeta=C1*sum1;
        dLdr=C2*sum2;
    end
end
