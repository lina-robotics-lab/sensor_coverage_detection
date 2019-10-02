% This file contains the class of eight_shaped_dynamics function.
classdef eight_shaped_dynamics < handle
    properties
        omega = 0.1;
    end
    methods
        function obj = eight_shaped_dynamics(omega)
          if nargin > 0
            obj.omega = omega;
          end
        end
        function loc = subsref(obj,input)
            t = input(1).subs{1};
            omega = obj.omega;
           loc = [sin(omega*t), sin(omega*t)*cos(omega*t)];
        end
    end
end