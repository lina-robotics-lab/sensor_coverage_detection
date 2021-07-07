limit = 10000;

time = 0;
dt = 0.01;
omega = 0.05*pi;
dynamics = EightShapeDynamics(omega,dt);
initial_location = [1e-5,1e-5]; % Caveat: do not choose a [0,0] initial location for 8-shape curve!
target = TargetClass(initial_location,@dynamics.stateAt);
target.incrementTime(dt);


loc = target.returnPos();
locs = loc;
for i = 1:limit
    % Testing stateUpdate utility in dynamics object.
    
    
    loc = dynamics.stateUpdate(loc);
    locs = [locs;loc];

    target.motionUpdate();
    target.incrementTime(dt);
end

% target.states
figure;
plot(target.states(1:end, 1), target.states(1:end, 2));
figure;
plot(locs(:,1),locs(:,2))