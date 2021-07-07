% This file contains the class of eight_shaped_dynamics function.
% When using this class, do the following:
% >> dynamics = EightShapeDynamics(omega,dt)
% >> dynamics.stateAt(t)
% >> dynamics.stateUpdate(state)
classdef Cheating_EightShapeDynamics < handle
    properties
        omega = 0.1; % omega is the revolving frequency of the curve.
        
        dt = 1; % dt is required by stateUpdate method. It is the time 
                % interval for stateUpdate.
        t = 0;
    end
    methods
        function obj = Cheating_EightShapeDynamics(omega,dt)
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
           state = [sin(omega*t); sin(omega*t)*cos(omega*t)];
        end
        function updateTime(obj)
           obj.t=obj.t+obj.dt;
        end
     
        function resetTime(obj)
           obj.t=0;
        end
        
        function new_state = stateUpdate(obj,state,varargin)
                 phase = obj.omega*obj.t;
                 new_state = [sin(phase);sin(phase)*cos(phase)];

        end
        
        function new_state = stateUpdateWithNoise(obj,state,varargin)
            global proc_noise_variance;
                
            % This method takes in a state vector generate by a
            % eight_shape_dynamics with the same omega parameter,
            % return the state vector at t+obj.dt
                 phase = obj.omega*obj.t;
                 new_state = [sin(phase);sin(phase)*cos(phase)];

                noise = randn(size(new_state))*proc_noise_variance;
                new_state = new_state+noise;
          
        end
    end
end
