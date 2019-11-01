classdef SensorClass < handle
   
    properties
        time = 0;
        angle = 0;
        states = [];    % track the angle along w/ x,y
        sigma = 0.1;        
        movement = [0, 0];
    end
    
    methods
        function obj = SensorClass(location, target)
            distance = location - target;
            obj.angle = atan2(distance(2), distance(1));%atan(location(2)/location(1));
            temp = [obj.states; [location, obj.angle, obj.time]];  
            obj.states = temp;
            obj.sigma = 0.001;
        end
        
        function r = incrementTime(obj, dt)
           obj.time = obj.time + dt; 
        end
        
        function r = returnPos(obj)
            r = obj.states(end, 1:2);
        end
        
        function r = returnAngle(obj)
            r = obj.states(end, 3);
        end
        
        function r = measureTarget(obj, target) % make angle calc to target
            location = obj.states(end, 1:2) - target;
            obj.angle = atan2(location(2), location(1));
            r = obj.angle;
        end
        
        % Finds angle between two sensors
        function r = angleFinder(obj, neighbor, own)
            ownQuat = eul2quat([0, own, 0]);
            ownQuat = ownQuat / norm(ownQuat);
            neighQuat = eul2quat([0, neighbor, 0]);
            neighQuat = neighQuat / norm(neighQuat);
            totQuat = quatmultiply(quatconj(ownQuat), neighQuat);
            
            dif = 2 * atan2(norm(totQuat(2:4)),totQuat(1));
            
%             dif = 0;
%             if(sign(neighbor) == sign(own))
%                 dif = own - neighbor;
%                 if(dif > 0)
%                    dif = 2*pi - dif; 
%                 end
%             else
%                dif = own - neighbor; 
%             end
%             
            if(dif < 0)
                dif = 2*pi + dif;
            end
            r = dif;          
        end
        
        function r = moveSensor(obj, cwNeighbor, ccwNeighbor)   % only angles to neighbors
%             obj.angle
%             cwNeighbor
%             ccwNeighbor
            cwDif = obj.angleFinder(cwNeighbor, obj.angle);
            ccwDif = obj.angleFinder(ccwNeighbor, obj.angle);
  
            
            angle_move = (1/4)*(cwDif - ccwDif);
                       
            obj.angle = obj.angle + angle_move;
            
            radius = 1;
            location = [radius*cos(obj.angle), radius*sin(obj.angle)];
            
            temp = [obj.states; [location, obj.angle, obj.time]];  
            obj.states = temp;
            
            r = location;
                        
        end
        
%         function r = moveSensor(obj, move)
%             temp = [obj.states; [location, obj.angle, obj.time]];  
%             obj.states = temp;
%         end
    end
    
end