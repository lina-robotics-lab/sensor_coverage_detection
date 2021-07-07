classdef StationaryDynamics < handle
    properties
    end
    methods
        function obj = StationaryDynamics(omega,dt)
        end
        
        function new_state = stateUpdate(obj,state,varargin)
            new_state=state;
        end
        
        function new_state = stateUpdateWithNoise(obj,state,varargin)
            global proc_noise_variance;
                
                noise = normrnd(0,sqrt(proc_noise_variance),size(state));
                new_state = state+noise;
        end
    end
end
