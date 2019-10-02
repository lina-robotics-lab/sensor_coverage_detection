classdef SensorClass < handle
   
    properties
        time = 0;
        angle = 0;
        states = [];    % track the angle along w/ x,y
        sigma = 0.1;        
    end
    
    methods
        function obj = SensorClass(location)
            angle = atan(location(2)/location(1));
            temp = [obj.states, [location, angle, obj.time]];  
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
            r = atan(location(2)/location(1));
        end
        
        function r = moveSensor(obj, cwNeighbor, ccwNeighbor)   % only angles to neighbors
            cwDif = abs(cwNeighbor - obj.angle);
            ccwDif = abs(ccwNeighbor - obj.angle);
            
            move 
            
        end
    end
    
end