classdef CircleShapeDynamics < handle
     properties
        omega = 0.01; % omega is the movement along the curve.
        
        dt = 0.1; % dt is required by stateUpdate method. It is the time 
                % interval for stateUpdate.
        t = 0;
        radius = 0.5;
                
     end
    
     methods
        function obj = CircleShapeDynamics(omega,dt)
          if nargin >= 1
            obj.omega = omega;
          end
          if nargin >= 2  
             obj.dt = dt;
          end
        end
        
        function state = stateAt(obj,t)
           % A state vector is a 
           omega = obj.omega;
           radius = obj.radius;
           state = [radius*cos(omega*t); radius*sin(omega*t)];
        end
        
        function new_state = stateUpdate(obj,state,varargin)
            % This method takes in a state vector generate by a
            % eight_shape_dynamics with the same omega parameter,
            % return the state vector at t+obj.dt
            if false
                disp("At location (0,0), the state update is undeterminable!");
            else
                omega = obj.omega;
                radius = obj.radius;
                dt = obj.dt;
                epsilon = 1e-8;

                % Magical math derivation.
                [theta, r] = cart2pol(state(1), state(2));
                 
                new_state = [state(1) - radius*omega*sin(theta)*dt; state(2) + radius*omega*cos(theta)*dt];
            end
        end
        
        function new_state = stateUpdateWithNoise(obj,state,varargin)
            global proc_noise_variance;
                
            % This method takes in a state vector generate by a
            % eight_shape_dynamics with the same omega parameter,
            % return the state vector at t+obj.dt
            if false
                disp("At location (0,0), the state update is undeterminable!");
            else
                omega = obj.omega;
                dt = obj.dt;
                epsilon = 1e-8;

                % Magical math derivation.
                [theta,r] = cart2pol(state(1), state(2));
                 
                new_state = [state(1) - radius*omega*sin(theta)*dt; state(2) + radius*omega*cos(theta)*dt];
                noise = randn(size(new_state))*proc_noise_variance;
                new_state = new_state+noise;
            end
        end
        
     end
     
end