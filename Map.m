limit = 10000;

time = 0;
dt = 0.01;
target = TargetClass([0,0]);
target.incrementTime(dt);

for i = 1:limit
%     target.motionUpdate();
%     target.incrementTime(dt);
      target.motionUpdateDefault(dt)
end

% target.states

plot(target.states(1:end, 1), target.states(1:end, 2));
