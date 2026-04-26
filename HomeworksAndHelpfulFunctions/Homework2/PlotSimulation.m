function PlotSimulation(time, aircraft_state_array, control_input_array, col)
%Author: Lucas Becker
%Date: 1/20/2026
% This function is used to plot the results of a complete simulation once it is completed. It
% takes as input the length n vector which holds the time corresponding to the set of variables,
% the 12 by n array of aircraft state, the 4 by n array of control inputs, and the string col
% which indicates the plotting option used for every plot, e.g. col = 'b−′.

%plotting positon versus time
figure(1)
hold on
subplot(311)
plot(time, aircraft_state_array(:, 1), col)
hold on
title('Aircraft X Position')
xlabel('Time (s)')
ylabel('X Position (m)')

subplot(312)
plot(time,aircraft_state_array(:,2), col)
hold on
title('Aircraft Y Position')
xlabel('Time (s)')
ylabel('Y Position (m)')

subplot(313)
plot(time, -aircraft_state_array(:, 3), col)
hold on
title("Aircraft Z Position")
xlabel('Time (s)')
ylabel('Z Position (m)')

%plotting orientation
figure(2)
hold on

subplot(311)
hold on
plot(time, aircraft_state_array(:, 4), col)
title('Aircraft Roll Angle')
xlabel('Time (s)')
ylabel('Roll Angle (rad)')

subplot(312)
hold on
plot(time, aircraft_state_array(:,5), col)
title('Aircraft Pitch Angle')
xlabel('Time (s)')
ylabel('Pitch Angle (rad)')

subplot(313)
hold on
plot(time, aircraft_state_array(:,6), col)
title('Aircraft Yaw Angle')
xlabel('Time (s)')
ylabel('Yaw Angle (rad)')

%Plotting velocities
figure(3)
hold on
subplot(311)
hold on
plot(time, aircraft_state_array(:,7), col)
title('Aircraft X Velocity')
xlabel('Time (s)')
ylabel('X Velocity (m/s)')

subplot(312)
hold on
plot(time, aircraft_state_array(:,8), col)
title('Aircraft Y Velocity')
xlabel('Time (s)')
ylabel('Y Velocity (m/s)')

subplot(313)
hold on
plot(time, aircraft_state_array(:,9), col)
title('Aircraft Z Velocity')
xlabel('Time (s)')
ylabel('Z Velocity (m/s)')

%Plotting Angular Velocities:
figure(4)
hold on
subplot(311)
hold on
plot(time, aircraft_state_array(:,10), col)
title('Aircraft Roll Rate')
xlabel('Time (s)')
ylabel('Roll Rate (rad/s)')

subplot(312)
hold on
plot(time, aircraft_state_array(:,11), col)
title('Aircraft Pitch Rate')
xlabel('Time (s)')
ylabel('Pitch Rate (rad/s)')

subplot(313)
hold on
plot(time, aircraft_state_array(:,12), col)
title('Aircraft Yaw Rate')
xlabel('Time (s)')
ylabel('Yaw Rate (rad/s)')

% Plotting control inputs
figure(5)
hold on
subplot(411)
hold on
plot(time, control_input_array(1,:), col)
title('Control Input 1')
xlabel('Time (s)')
ylabel('Input Value')
axis padded
grid on

subplot(412)
hold on
plot(time, control_input_array(2,:), col)
title('Control Input 2')
xlabel('Time (s)')
ylabel('Input Value')
axis padded
grid on

subplot
subplot(413)
hold on
plot(time, control_input_array(3,:), col)
title('Control Input 3')
xlabel('Time (s)')
ylabel('Input Value')
axis padded
grid on

subplot(414)
hold on
plot(time, control_input_array(4,:), col)
title('Control Input 4')
xlabel('Time (s)')
ylabel('Input Value')
axis padded
grid on

%plotting three dimensional track
figure(6)
hold on
plot3(aircraft_state_array(:, 1), aircraft_state_array(:, 2), -aircraft_state_array(:,3), col)
plot3(aircraft_state_array(1,1),aircraft_state_array(1,2),-aircraft_state_array(1,3),'o','MarkerEdgeColor','g')
plot3(aircraft_state_array(end, 1), aircraft_state_array(end, 2), -aircraft_state_array(end,3), 'x','MarkerEdgeColor','r')
title('3D Flight Path')
xlabel('X Position (m)')
ylabel('Y Position (m)')
zlabel('Z Position (m)')
grid on



% Plotting all 12 states on one figure using subplots
figure(7)
hold on

state_titles = { ...
    'X Position', ...
    'Y Position', ...
    'Z Position', ...
    'Roll Angle', ...
    'Pitch Angle', ...
    'Yaw Angle', ...
    'X Velocity', ...
    'Y Velocity', ...
    'Z Velocity', ...
    'Roll Rate', ...
    'Pitch Rate', ...
    'Yaw Rate'};

for i = 1:12
    subplot(6,2,i)
    hold on
    plot(time, aircraft_state_array(:,i), col)
    title(state_titles{i})
    xlabel('Time (s)')
    ylabel('State Value')
end



end