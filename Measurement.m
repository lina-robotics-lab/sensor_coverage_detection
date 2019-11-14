classdef Measurement < handle
    % Measurement function does not involve noises.
    properties
        % The parameters of the general measurement model used by Martinez and Bullo.
        R1= 1e5;
        R0 = 1e-5;
        % There must be R1>R0>0
        
        c1 = 0;
        c2 = 0;
        b = 1;
        
        sensorLocs = []; 
        % sensorLocs is an array of sensor location required by
        % measureUpdate method.
        
        measure_noise = 0.0005;
    end
    
    methods
        
        function obj=Measurement(b,measure_noise,c1,c2,R1,R0)
            if exist('b','var')
              obj.b = b;            
            end
            if exist('c1','var')
              obj.c1 = c1;            
            end
            if exist('c2','var')
              obj.c2 = c2;            
            end
            if exist('R1','var')
              obj.R1 = R1;            
            end
            if exist('R0','var')
              obj.R0 = R0;            
            end
            if exist('measure_noise','var')
              obj.measure_noise = measure_noise;            
            end
        end
        
        function h = measure(obj,r,noisy)
            % noisy is a 0-1 variable which decides whether the measurement
            % outcome should contain noise.
            
            % The formula for general measurement function used in the
            % Martinez, Bullo paper.
            r = min(max(r,obj.R0),obj.R1);
            
            h = (r-obj.c1)^obj.b+obj.c2 + noisy*normrnd(0, sqrt(obj.measure_noise));
            h = max(h,0);% Ensure a non-negative value is returned.
        end
        
        function y = measureUpdatePerfect(obj,state,varargin)
            % No measurement noise is generated. This is the function
            % handle to pass to the EKF.
            
            % Given a state vector(essentially a location vector),
            % return the vector of perfect measurement
            % output from sensors at sensorLocs, using the measure() method
            % of this class.
            
            
            % We assume sensorLocs are row vectors, state is a column vector.
            % The measurement output is a column vector.
    
            rs = sqrt(sum((obj.sensorLocs-state).^2,1))';
            y = zeros(length(rs),1);
            for i=1:length(y)
                y(i)=obj.measure(rs(i),false);% No measurement noise is generated.
            end
        end
        function y = measureUpdateWithNoise(obj,state,varargin)
            % Measurement noise will be added. Only use this method to
            % generate measurement data in the simulation.
            % Do NOT pass this as a function handle to the EKF.
            
            % Given a state vector(essentially a location vector),
            % return the vector of measurement
            % output from sensors at sensorLocs, using the measure() method
            % of this class.
            
            % We assume sensorLocs are row vectors, state is a column vector.
            % The measurement output is a column vector.
    
            rs = sqrt(sum((obj.sensorLocs-state).^2,1))';
            y = zeros(length(rs),1);
            for i=1:length(y)
                y(i)=obj.measure(rs(i),true);    % Measurement noise will be added. Only use this method to
            % generate measurement data in the simulation.
            end
        end
        
        function dhdq = measureJacobian(obj,state,varargin)
            b= obj.b;
            rs = sqrt(sum((obj.sensorLocs-state).^2,1))';
            dhdq=b*rs.^(b-2).* (state-obj.sensorLocs)';
        end
        
    end
end
 