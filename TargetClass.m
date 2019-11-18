 classdef TargetClass < handle  
    properties
        time = 0;
        states = [];
        
        % sigma is the std parameter for Gaussian Noise magnitude.
        sigma = 0.1;

        % dynamics should be a function which takes in a single parameter, t, and return a location vector. 
        dynamics=[]; 
       
       
    end
    
    methods
        function obj = TargetClass(location,dynamics)
            temp = [obj.states, location];  
            obj.states = temp;
            obj.sigma = 0.01;
            if nargin<2
                % default target dynamics, 8-shape curve
                omega = 0.1;
                obj.dynamics = @(t) [sin(omega*t), sin(omega*t)*cos(omega*t)];
            else
                obj.dynamics = dynamics;
            end

        end

        function r = incrementTime(obj, dt)
           obj.time = obj.time + dt; 
        end
        
        function r = returnPos(obj)
            r = obj.states(end, 1:2);
        end

        function r = motionUpdate(obj)

            next = obj.dynamics(obj.time);
            next = next + normrnd(0, obj.sigma, [1,2]);
            temp = [obj.states; next]; 
            obj.states = temp;
            
            r = obj.states(end, 1:2);
        end
    end
end