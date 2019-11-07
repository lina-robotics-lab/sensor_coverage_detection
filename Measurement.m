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
        
    end
    
    methods
        
        function obj=Measurement(b,c1,c2,R1,R0)
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
            
        end
        
        function h = measure(obj,r)
%             global measure_noise_mag;
            % The formula for general measurement function used in the
            % Martinez, Bullo paper.
            r = min(max(r,obj.R0),obj.R1);
            
            h = (r-obj.c1)^obj.b+obj.c2;
%             h=h+randn(1)*measure_noise_mag;
%             h=h+randn(1)*5e-2;
        end
        
        function y = measureUpdate(obj,state,varargin)
            % Given a state vector(essentially a location vector),
            % return the vector of measurement
            % output from sensors at sensorLocs, using the measure() method
            % of this class.
            
            % We assume sensorLocs are row vectors, state is a column vector.
            % The measurement output is a column vector.
    
            rs = sqrt(sum((obj.sensorLocs-state).^2,1))';
            y = zeros(length(rs),1);
            for i=1:length(y)
                y(i)=obj.measure(rs(i));
            end
        end
        function dhdq = measureJacobian(obj,state,varargin)
            b= obj.b;
            rs = sqrt(sum((obj.sensorLocs-state).^2,1))';
            dhdq=b*rs.^(b-2).* (state-obj.sensorLocs)';
        end
        
    end
end
 