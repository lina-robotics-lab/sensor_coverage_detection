classdef SensorClass < handle
   
    properties
        time = 0;
        % Implementation conventions: 
        % 1. Angles should always vary between 0 and 2*pi
        % 2. Angle value increases in counter-clockwise(ccw) direction.
        angle_to_target = NaN;
        
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
            
            
%             [theta,rho]= cart2pol(rel_loc(1), rel_loc(2));
%             obj.angle_to_target=mod(theta,2*pi);%angles should always vary between 0 and 2*pi
%             
            temp = [obj.states; [location, obj.angle_to_target, obj.time]];  
            obj.states = temp;
      
            obj.k=1/4;
            if nargin>=3
                obj.k = varargin{1};
            end
            
        end
        
        function r = incrementTime(obj, dt)
           obj.time = obj.time + dt; 
        end
        
        function r = returnPos(obj)
            r = obj.states(end, 1:2);
        end
        
        function r = returnAngle(obj)
            r = obj.states(end, 3);
        end
        
        function r = measureTarget(obj, target) % make angle calc to target
            rel_loc_target = obj.states(end, 1:2) - target;
            
            %Syntax:[theta,rho]=cart2pol(x,y)
            [theta,rho]= cart2pol(rel_loc_target(1), rel_loc_target(2));%cart2pol returns an angle between -pi and pi
            obj.angle_to_target=mod(theta,2*pi);%angles should always vary between 0 and 2*pi
            
            temp = [obj.states; [rel_loc_target, obj.angle_to_target, obj.time]];  
            obj.states = temp;
          
            r = obj.angle_to_target;
        end
        
        function r = angleFinder(obj, neighbor_angle, own_angle)
            % Both neighbor_angle and own_angle are between 0 and 2*pi.
            % The angle difference is always between 0 and pi.
            diff=abs(neighbor_angle-own_angle);
            r=min(2*pi-diff,diff);
        end

        function r = moveSensor(obj, cwNeighbor, ccwNeighbor,target_loc)   % only angles to neighbors
            cwDif = obj.angleFinder(cwNeighbor, obj.angle_to_target);
            ccwDif = obj.angleFinder(ccwNeighbor, obj.angle_to_target);
             
            angle_move = obj.k*(ccwDif - cwDif);
            % This is the rule specifie by Martinez & Bullo's paper.
            % k is the control gain. Different k will influence the final
            % convergence locations.
            
            % The angle must vary between 0 and 2*pi.
            obj.angle_to_target = mod(obj.angle_to_target + angle_move,2*pi); 
            
            % Caculate the extended update location as if the sensor can
            % move out of the boundary.
            ext_r = dist(obj.states(end,1:2),target_loc);
            extended_update = target_loc + ext_r*[cos(obj.angle_to_target), sin(obj.angle_to_target)];
            
            % Project the extended location back to the boundary.
            rel_location = extended_update - obj.boundary_origin;
            rel_dist = dist(extended_update,obj.boundary_origin);
            location = obj.boundary_origin+obj.boundary_r*rel_location/rel_dist; 
            
            temp = [obj.states; [location, obj.angle_to_target, obj.time]];  
            obj.states = temp;
            
            r = location;
                        
        end
        
%         function r = moveSensor(obj, move)
%             temp = [obj.states; [location, obj.angle_to_target, obj.time]];  
%             obj.states = temp;
%         end
    end
    
end
function r=dist(x,y)
    r=sum((x-y).^2).^0.5;
end