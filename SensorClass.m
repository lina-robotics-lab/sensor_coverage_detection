classdef SensorClass < handle
   
    properties
        time = 0;
        % Implementation conventions: 
        % 1. Angles should always vary between 0 and 2*pi
        % 2. Angle value increases in counter-clockwise(ccw) direction.
        angle = 0;
        states = [];    % track the angle along w/ x,y
        sigma = 0.1;        
        movement = [0, 0];
    end
    
    methods
        function obj = SensorClass(location, target)
            distance = location - target;
            [theta,rho]= cart2pol(distance(1), distance(2));
            obj.angle=mod(theta,2*pi);%angles should always vary between 0 and 2*pi
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
            
            %[theta,rho]=cart2pol(x,y)
            [theta,rho]= cart2pol(location(1), location(2));%cart2pol returns an angle between -pi and pi
            obj.angle=mod(theta,2*pi);%angles should always vary between 0 and 2*pi
            r = obj.angle;
        end
        
        function r = angleFinder(obj, neighbor_angle, own_angle)
            % Both neighbor_angle and own_angle are between 0 and 2*pi.
            % The angle difference is always between 0 and pi.
            diff=abs(neighbor_angle-own_angle);
            r=min(2*pi-diff,diff);
        end

        function r = moveSensor(obj, cwNeighbor, ccwNeighbor)   % only angles to neighbors
            cwDif = obj.angleFinder(cwNeighbor, obj.angle);
            ccwDif = obj.angleFinder(ccwNeighbor, obj.angle);
             
            angle_move = (1/4)*(ccwDif - cwDif);% This is the rule specifie by Martinez & Bullo's paper
            
            % The angle must vary between 0 and 2*pi.
            obj.angle = mod(obj.angle + angle_move,2*pi); 
            
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