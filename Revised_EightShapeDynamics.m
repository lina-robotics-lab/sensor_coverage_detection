% This file contains the class of eight_shaped_dynamics function.
% When using this class, do the following:
% >> dynamics = EightShapeDynamics(omega,dt)
% >> dynamics.stateAt(t)
% >> dynamics.stateUpdate(state)
classdef Revised_EightShapeDynamics < handle
    properties
        omega = 0.1; % omega is the revolving frequency of the curve.
        
        dt = 0.1; % dt is required by stateUpdate method. It is the time 
                % interval for stateUpdate.
        
    end
    methods
        function obj = Revised_EightShapeDynamics(omega,dt)
          if nargin >= 1
            obj.omega = omega;
          end
          if nargin >= 2  
             obj.dt = dt;
          end
        end
        function state = stateAt(obj,t)
           % A state vector is a column vector
           omega = obj.omega;
           state = [omega*t;sin(omega*t); sin(omega*t)*cos(omega*t)];
        end
        
        function new_state = stateUpdate(obj,state,varargin)
            % This method takes in a state vector generate by a
            % eight_shape_dynamics with the same omega parameter,
            % return the state vector at t+obj.dt
                omega = obj.omega;
                dt = obj.dt;
                theta = state(1);
                new_theta = theta+omega*dt;
                new_state=[new_theta;sin(new_theta);sin(new_theta)*cos(new_theta)];
            
        end
        
        function new_state = stateUpdateWithNoise(obj,state,varargin)
            global proc_noise_variance;
                
            % This method takes in a state vector generate by a
            % eight_shape_dynamics with the same omega parameter,
            % return the state vector at t+obj.dt
            
            theta = state(1);
            new_theta = theta+omega*dt;
            new_state=[new_theta;sin(new_theta);sin(new_theta)*cos(new_theta)];
            noise = randn(size(new_state))*proc_noise_variance;
            new_state = new_state+noise;
            
        end
    end
end
