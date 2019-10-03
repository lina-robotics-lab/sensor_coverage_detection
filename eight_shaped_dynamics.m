% This file contains the class of eight_shaped_dynamics function.
% When using this class, do the following:
% >> dynamics = eight_shaped_dynamics(omega,dt)
% >> dynamics.stateAt(t)
% >> dynamics.stateUpdate(state)
classdef eight_shaped_dynamics < handle
    properties
        omega = 0.1; % omega is the revolving frequency of the curve.
        
        dt = 0.1; % dt is required by stateUpdate method. It is the time 
                % interval for stateUpdate.
        
    end
    methods
        function obj = eight_shaped_dynamics(omega,dt)
          if nargin == 1
            obj.omega = omega;
          elseif nargin == 2  
             obj.dt = dt;
          end
        end
        function state = stateAt(obj,t)
           omega = obj.omega;
           state = [sin(omega*t), sin(omega*t)*cos(omega*t)];
        end
        
        function new_state = stateUpdate(obj,state)
            % This method takes in a state vector generate by a
            % eight_shape_dynamics with the same omega parameter,
            % retrieve t from the state vector, and then return the state vector at t+obj.dt
            
            if state(1)==0
                disp("sin(omega*t)=0, the state update is undeterminable!");
            else
                omega = obj.omega;
                dt = obj.dt;
                epsilon = 1e-8;

                % Magical math derivation. Relies on the non-zero assumption of
                % state(1), i.e. sin(omega*t).
                new_state = [state(1)*cos(omega*dt)+sin(omega*dt)*state(2)/state(1),state(2)*cos(2*omega*dt)+(1/2-state(1)^2)*sin(2*omega*dt)];
            end
        end
    end
end