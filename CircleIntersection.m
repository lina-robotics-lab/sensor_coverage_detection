target_loc=[0,0];
target_angle=1.4*pi;
boundary_origin=[0.5,0];
boundary_radius=2;
locs=CircleIntersections(target_loc,target_angle,boundary_origin,boundary_radius);

x = -4:0.01:4;
target_line = tan(target_angle)*(x-target_loc(1))+target_loc(2);
plot(x,target_line,'DisplayName',"target direction");
hold on;
scatter(target_loc(1),target_loc(2),'filled','DisplayName',"target");
scatter(real(locs(1)),imag(locs(1)));
% hold on;
% scatter(real(locs(2)),imag(locs(2)));
legend();

thetas = 0:0.1:2*pi;
xs = boundary_origin(1)+boundary_radius.*cos(thetas);
ys = boundary_origin(2)+boundary_radius.*sin(thetas);
plot(xs,ys);
function intersections=CircleIntersections(target_loc,target_angle,boundary_origin,boundary_radius)
    o = boundary_origin(1)+boundary_origin(2)*1j;
    t=target_loc(1)+target_loc(2)*1j;
    ot = (o-t)*exp(-1j*target_angle);
    im = imag(ot)/boundary_radius;
    alpha = real(ot)+boundary_radius*sqrt(1-im^2);
    if imag(alpha)==0
        intersections = t+alpha*exp(1j*target_angle);
    else
        disp("NO SOLUTION! The target line does not intersect with the circle.");
        intersections=[];
    end
end
