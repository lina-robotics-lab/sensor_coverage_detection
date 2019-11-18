
% This function returns the intersection point of a half-line, which starts from
% target_loc in the direction of target_angle, with a circle, centered at
% boundary_origin with radius=boundary_radius. The derivation of the
% formula requires complex number manipulations, which can be found in the Solve_For_Intersection.png in the project
% repository.
function intersection=CircleIntersection(target_loc,target_angle,boundary_origin,boundary_radius)
    o = boundary_origin(1)+boundary_origin(2)*1j;
    t=target_loc(1)+target_loc(2)*1j;
    ot = (o-t)*exp(-1j*target_angle);
    im = imag(ot)/boundary_radius;
    alpha = real(ot)+boundary_radius*sqrt(1-im^2);
    if imag(alpha)==0
        intersection = t+alpha*exp(1j*target_angle);
        intersection=[real(intersection);imag(intersection)];
    else
%         disp("NO SOLUTION! The target line does not intersect with the circle.");
        intersection=[];
    end
end
