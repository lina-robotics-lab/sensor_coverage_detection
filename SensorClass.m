classdef SensorClass < handle
   
    properties
        time = 0;
        % Implementation conventions: 
        % 1. Angles should always vary between 0 and 2*pi
        % 2. Angle value increases in counter-clockwise(ccw) direction.
        % 3. Location vector is a column vector.
        angle_to_target = NaN;
        curr_loc=[0;0];
        
        % A row in obj.states is [x,y,angle,time].
        states = [];    % track the angle along w/ x,y
        
        % Control gain as defined in Martinez & Bullo's Paper. By default
        % it takes value 1/4.
        k = 1/4;         
        
        boundary_origin=[0,0];
        boundary_r = 1.5;
    end
    
    methods
        function obj = SensorClass(location, boundary_origin,boundary_r,varargin)
            obj.boundary_origin=boundary_origin;
            obj.boundary_r = boundary_r;
            obj.curr_loc =location;
            obj.states = [obj.states; [obj.curr_loc', obj.angle_to_target, obj.time]];        
            
            obj.k=1/4;
            if nargin>=3
                obj.k = varargin{1};
            end
            
        end
        
        function r = incrementTime(obj, dt)
           obj.time = obj.time + dt; 
        end
        
        function r = returnPos(obj)
            r = obj.curr_loc;
        end
        
        function r = returnAngle(obj)
            r = obj.angle_to_target;
        end
        
        function r = measureTarget(obj, target) % make angle calc to target
            rel_loc_target = obj.curr_loc - target;
            
            %Syntax:[theta,rho]=cart2pol(x,y)
            [theta,rho]= cart2pol(rel_loc_target(1), rel_loc_target(2));%cart2pol returns an angle between -pi and pi
            obj.angle_to_target=mod(theta,2*pi);%angles should always vary between 0 and 2*pi
            
            obj.states = [obj.states; [obj.curr_loc', obj.angle_to_target, obj.time]];  
          
            r = obj.angle_to_target;
        end
        
        function r = angleFinder(obj, neighbor_angle, own_angle)
            % Both neighbor_angle and own_angle are between 0 and 2*pi.
            diff=abs(neighbor_angle-own_angle);
            r=min(2*pi-diff,diff); % The angle difference r is always between 0 and pi.
        end
        
        function r=moveSensorGradient(obj,gradient,step_size)
		epsilon=1e-8;
            if norm(gradient)~=0
                obj.curr_loc=obj.curr_loc+step_size*gradient/(epsilon+norm(gradient));
%                 obj.curr_loc=obj.curr_loc+step_size*gradient;
            end
            assert(~isnan(obj.curr_loc(1)));
            obj.states =  [obj.states; [obj.curr_loc', NaN, obj.time]];
            r = obj.curr_loc;
        end
        function r=moveSensorEquiAngular(obj,cwNeighbor, ccwNeighbor,target_loc)
            cwDif = obj.angleFinder(cwNeighbor, obj.angle_to_target);
            ccwDif = obj.angleFinder(ccwNeighbor, obj.angle_to_target);
             
            angle_move = obj.k*(ccwDif - cwDif);
            % This is the rule specifie by Martinez & Bullo's paper.
            % k is the control gain. Different k will influence the final
            % convergence locations.
            
            % The angle must vary between 0 and 2*pi.
            angle_update = mod(obj.angle_to_target + angle_move,2*pi); 
            
            % Caculate the update location, subject to that the sensor must
            % move on the boundary.
            
            locs = CircleIntersection(target_loc,angle_update,obj.boundary_origin,obj.boundary_r);
            if ~isempty(locs)
               obj.curr_loc=locs;
            end
            % Note: we can simulate giving a control signal to the sensor
            % by directly updating the location, but we cannot claim
            % anything about the angle_to_target state here, since the
            % target may also be moving. So we set it to be NaN.
            obj.states =  [obj.states; [obj.curr_loc', NaN, obj.time]];
            r = obj.curr_loc;
        end
        
    end
    
end
