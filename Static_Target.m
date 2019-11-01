close all;

s1 = SensorClass([0, 1], [0,0]);
s1.measureTarget([0,0]);
s2 = SensorClass([0.05, 0.9987], [0,0]);
% s2 = SensorClass([1, 0], [0,0]);
s2.measureTarget([0,0]);
s3 = SensorClass([-0.05, 0.9987], [0,0]);
% s3 = SensorClass([-1, 0], [0,0]);
s3.measureTarget([0,0]);

sensors = [s1, s2, s3];

for i = 1:50
   angles = [s3.measureTarget([0,0]), s1.measureTarget([0,0]), s2.measureTarget([0,0]), s3.measureTarget([0,0]), s1.measureTarget([0,0])];
   for j = 1:length(sensors)
      sensors(j).moveSensor(angles(j+2), angles(j)); 
   end
%    disp("derp");
end

out = [s1.returnAngle(), s2.returnAngle(), s3.returnAngle()]

% hold on;
% scatter3(s1.states(1:end, 1), s1.states(1:end, 2), 1:1:51);
% scatter3(s2.states(1:end, 1), s2.states(1:end, 2), 1:1:51);
scatter3(s3.states(1:end, 1), s3.states(1:end, 2), 1:1:51);