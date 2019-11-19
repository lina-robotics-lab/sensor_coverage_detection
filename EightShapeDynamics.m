% This file contains the class of eight_shaped_dynamics function.
% When using this class, do the following:
% >> dynamics = EightShapeDynamics(omega,dt)
% >> dynamics.stateAt(t)
% >> dynamics.stateUpdate(state)
classdef EightShapeDynamics < handle
    properties
        omega = 0.1; % omega is the revolving frequency of the curve.
        
        dt = 1; % dt is required by stateUpdate method. It is the time 
                % interval for stateUpdate.
        t = 0;
    end
    methods
        function obj = EightShapeDynamics(omega,dt)
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
        
        function new_state = stateUpdate(obj,state,varargin)
            % This method takes in a state vector generate by a
            % eight_shape_dynamics with the same omega parameter,
            % return the state vector at t+obj.dt
                omega = obj.omega;
                dt = obj.dt;
                epsilon = 1e-8;

                % Magical math derivation.
                % Key formula used:sin(a+b)=sin(a)cos(b)+cos(a)sin(b).
                if state(1)==0
                    new_state = [sin(omega*dt);sin(omega*dt)*cos(omega*dt)];
                else
                    
                    new_state = [state(1)*cos(omega*dt)+sin(omega*dt)*abs(cos(asin(state(1))))*sign(state(1)*state(2));state(2)*cos(2*omega*dt)+(1/2-state(1)^2)*sin(2*omega*dt)];
%                     new_state = [state(1)*cos(omega*dt)+sin(omega*dt)*(state(2))/(state(1)+epsilon) ;state(2)*cos(2*omega*dt)+(1/2-state(1)^2)*sin(2*omega*dt)];
                end
%                  new_state = [state(1)*cos(omega*dt)+sin(omega*dt)*(state(2))/(state(1)+epsilon) ;state(2)*cos(2*omega*dt)+(1/2-state(1)^2)*sin(2*omega*dt)];
             
            
        end
        
        function new_state = stateUpdateWithNoise(obj,state,varargin)
            global proc_noise_variance;
                
            % This method takes in a state vector generate by a
            % eight_shape_dynamics with the same omega parameter,
            % return the state vector at t+obj.dt
                omega = obj.omega;
                dt = obj.dt;
                epsilon = 1e-8;

                % Magical math derivation.
                % Key formula used:sin(a+b)=sin(a)cos(b)+cos(a)sin(b).
                if state(1)==0
                    new_state = [sin(omega*dt);sin(omega*dt)*cos(omega*dt)];
                else
                  new_state = [state(1)*cos(omega*dt)+sin(omega*dt)*abs(cos(asin(state(1))))*sign(state(1)*state(2));state(2)*cos(2*omega*dt)+(1/2-state(1)^2)*sin(2*omega*dt)];
%                                   new_state = [state(1)*cos(omega*dt)+sin(omega*dt)*(state(2))/(state(1)+epsilon) ;state(2)*cos(2*omega*dt)+(1/2-state(1)^2)*sin(2*omega*dt)];

                end              
%                 new_state = [state(1)*cos(omega*dt)+sin(omega*dt)*(state(2))/(state(1)+epsilon) ;state(2)*cos(2*omega*dt)+(1/2-state(1)^2)*sin(2*omega*dt)];
                noise = normrnd(0,sqrt(proc_noise_variance),size(new_state));
                new_state = new_state+noise;
          
        end
    end
end
