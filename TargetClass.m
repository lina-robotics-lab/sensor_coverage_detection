classdef TargetClass < handle
    
    properties
        time = 0;
        states = [];
        sigma = 0.1;
    end
    
    methods
        function obj = TargetClass(location)
            temp = [obj.states, location];  
            obj.states = temp;
            obj.sigma = 0.001;
        end

        function r = incrementTime(obj, dt)
           obj.time = obj.time + dt; 
        end
        
        function r = returnPos(obj)
            r = obj.states(end, 1:2);
        end
        
        function r = motionUpdate(obj)

            next = [sin(obj.time/(8*pi)), sin(obj.time/(8*pi))*cos(obj.time/(8*pi))];
            next = next + normrnd(0, obj.sigma, [1,2]);
            temp = [obj.states; next]; 
            obj.states = temp;
            
            r = obj.states(end, 1:2);
        end
    end
end