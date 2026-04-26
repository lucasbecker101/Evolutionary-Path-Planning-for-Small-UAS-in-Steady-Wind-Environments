function wind_inertial = SensorsToWind(velocity_inertial, euler_angles, Va, beta, alpha, time) 
% Author: Lucas Becker
% Date Created: 3/25/2026
% ASEN 5128
% function to take in measurements from sensors and compute the wind vector
% in the inertial frame at every time step
% Also built to plot data to match homework 9 problem 2, but can be
% commented out

for i = 1:length(velocity_inertial)
% use wind angles to find the air relative velocity vector at every time
% step:
air_relative_velocity_body(:,i) = WindAnglesToAirRelativeVelocityVector([Va(i); beta(i); alpha(i)]);

%convert to inertial:
air_relative_velocity_inertial(:,i) = TransformFromBodyToInertial(air_relative_velocity_body(:,i),euler_angles(:,i));

%determine inertial velocity of wind:
wind_inertial(:,i) = velocity_inertial(:,i) - air_relative_velocity_inertial(:,i);
end


% ----------- Plotting for HW9 ----------- %

%air relative velocity vector expressed in body coordinates:
figure(1)
hold on 

subplot(311)
plot(time', air_relative_velocity_body(1,:))
title('Air Relative Velocity in Body Frame');
xlabel('Time (min)');
ylabel('x-component (m/s)');

subplot(312)
plot(time', air_relative_velocity_body(2,:))
xlabel('Time (min)');
ylabel('y-component (m/s)');

subplot(313)
plot(time', air_relative_velocity_body(3,:));
xlabel('Time (min)');
ylabel('z-component (m/s)');

% air relative velocity vector in inertial frame
figure(2)
hold on 

subplot(311)
plot(time', air_relative_velocity_inertial(1,:))
title('Air Relative Velocity in Inertial Frame');
xlabel('Time (min)');
ylabel('N-component (m/s)');

subplot(312)
plot(time', air_relative_velocity_inertial(2,:))
xlabel('Time (min)');
ylabel('E-component (m/s)');

subplot(313)
plot(time', air_relative_velocity_inertial(3,:));
xlabel('Time (min)');
ylabel('D-component (m/s)');


% wind vector in inertial frame
figure(3)
hold on 

subplot(311)
plot(time', wind_inertial(1,:))
title('Wind Velocity in Inertial Frame');
xlabel('Time (min)');
ylabel('N-component (m/s)');

subplot(312)
plot(time', wind_inertial(2,:))
xlabel('Time (min)');
ylabel('E-component (m/s)');

subplot(313)
plot(time', wind_inertial(3,:));
xlabel('Time (min)');
ylabel('D-component (m/s)');


end