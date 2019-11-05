classdef SensorClass < handle
   
    properties
        time = 0;
        angle = 0;
        states = [];    % track the angle along w/ x,y
        sigma = 0.1;        
    end
    
    methods
        % initializer
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
        
        % calculate current angle using the location of the object, for
        % better simulation accuracy
        function r = measureTarget(obj, target) % make angle calc to target
            location = obj.states(end, 1:2) - target;
            obj.angle = atan2(location(2), location(1));
            r = obj.angle;
        end
        
        % Finds angle between two sensors
        function r = angleFinder(obj, neighbor, own, clock)

            % Rotate the half-plane of the x axis to put the own
            % measurement on the axis, make measurement in that form
            dif = neighbor - own;
%             if(neighbor > 0 && dif < -pi)
%                 dif = 2*pi - dif; 
%             elseif(neighbor < 0 && dif > pi)
%                 dif = dif - 2*pi;
%             end
            
            % if the system is clockwise
            if(clock)
                if(sign(dif) < 0)   % if the target is in negative half-plane
                    r = abs(dif); 
                else
                    r = 2*pi - dif; % it flipped sides, add offset
                end
            else
                if(sign(dif) > 0)   % if the target is in positive half-plane
                    r = abs(dif);
                else
                    r = 2*pi + dif; % it flipped sides, add offset
                end
            end

        end
        
        function r = moveSensor(obj, cwNeighbor, ccwNeighbor)   % only angles to neighbors
            cwDif = obj.angleFinder(cwNeighbor, obj.angle, 1);
            ccwDif = obj.angleFinder(ccwNeighbor, obj.angle, 0);
            
            angle_move = (1/4)*(ccwDif - cwDif);    % move the agent
                       
            obj.angle = obj.angle + angle_move;
            if(abs(obj.angle) > pi) % if the agent has flipped past the pi/-pi line
                obj.angle = sign(obj.angle)*(abs(obj.angle) - 2*pi);
            end
            
            radius = 1;
            location = [radius*cos(obj.angle), radius*sin(obj.angle)];
            
            temp = [obj.states; [location, obj.angle, obj.time]];  
            obj.states = temp;
            
            r = location;
                        
        end
        
    end
    
end