% Lucas Becker
% ASEN 5128
% Homework8Main.m
% Edited: 3/11/2026
% Simulating TTwistor guidance models for ASEN 5128 Homework 8
clc; clear;
close all;  %comment out for plotting on same figure

%adding all functions created for 5128 to path
addpath(genpath('C:\Users\lucas\OneDrive\Documents\Current Classes\sUAS_GNC\Code'))

%initializing Ttwistor aircraft parameters
ttwistor

%loading ttwistor gains for both autopilots from homework 7
load("ttwistor_gains_feed");



%defnining trim
V_trim = 18;
h_trim = 1805;
gamma_trim = 0;
trim_definition = [V_trim; gamma_trim; h_trim];

%calculating trim for ttwistor at given trim definition
[x_star,u_star,trim_variables,fval] =  calculate_trim_level(trim_definition, aircraft_parameters);

%simulation paramaters:
t0 = 0;
tf = 200;
tspan = t0:0.1:tf;
wind_inertial = [0;0;0];

% ------------ PROBLEM 1 ------------ %
% problem 1:
%simulate using kinematic guidance model and compare to full aircraft
%dynamics and try getting model to match

% x = [pn pe chi chi_dot h h_dot Va]
% wind_inertial = [wn, we, wd]  (northeastdown wind)
% u_objective = [h_c; h_dot_c; chi_c; chi_dot_c; Va_c]
% params:
%   bcd, bc, bhd, bh, bVa   (tuneable guidance constants, start with 1) 

%setting guidance parameters (tuned in separate script)
guidance_params.bcd = 0.5;
guidance_params.bc = 0.22;
guidance_params.bhd = 0.18;
guidance_params.bh = 0.01;
guidance_params.bVa = 0.13;

% ***Submit plots from Homework8Tuning.m for this problem*** %


%defining the line to follow:
X_line = deg2rad(45);
gamma_line = deg2rad(10);
r = [200; 0; -h_trim];
q = [cos(gamma_line) * cos(X_line); cos(gamma_line) * sin(X_line); -sin(gamma_line)];
line_speed = V_trim;
d_line = linspace(0, line_speed * tf, 100);

n_pts_line = r(1) + d_line * q(1);
e_pts_line = r(2) + d_line * q(2);
h_pts_line = -r(3) + d_line * -q(3);

%guidance tuning parameters:
line_params.X_inf = deg2rad(30);
line_params.k_path = 0.01;

% ------------ PROBLEM 2 ------------ %
% problem 2:
% simulate straight line flight (to a specific straight line) using
% FirstOrderStraightGuidance
x_init2 = x_star(1:3,1);
[t_sim2,y_sim2] = ode45(@ (t,x) FirstOrderLineGuidance(t,x,line_speed,r,q,line_params),tspan,x_init2(1:3));

% plot for problem 2:
% 
% figure(2)
% hold on
% plot3(y_sim2(:,1),y_sim2(:,2),-y_sim2(:,3))
% plot3(n_pts_line,e_pts_line,h_pts_line,'r--')
% title('Problem 3')
% xlabel('North Position (m)')
% ylabel('East Position (m)')
% zlabel('Height (m)')
% legend('Aicraft','Intended Path')




% ------------ PROBLEM 3 ------------ %
% problem 3:
% guidance model: StraightLineGuidance, use the loop similar to hw7 to
% calculate guidance command inside loop and simulate with the
% kinematicguidancemodel developed for problem 1
Ts = .1;
n_ind = tf/Ts;

x_init3 = [0;0;0;0;1805;0;V_trim];
x_array3(:,1) = x_init3; 
time_iter3(1) = 0;

for i=1:n_ind

    TSPAN = Ts*[i-1 i];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Guidance level commands
    %%%
    %%%   control_objectives(1) = h_c;
    %%%   control_objectives(2) = h_dot_c;
    %%%   control_objectives(3) = chi_c;
    %%%   control_objectives(4) = chi_dot_ff;
    %%%   control_objectives(5) = Va_c;
    %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % STUDENTS WRITE THIS FUNCTION
    pos_array3(1,i) = x_array3(1,i);
    pos_array3(2,i) = x_array3(2,i);
    pos_array3(3,i) = -x_array3(5,i);
    control_objectives4 = StraightLineGuidance(pos_array3(:,i), line_speed, r, q, line_params ); 
    
    % x = [pn pe chi chi_dot h h_dot Va]

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%  Kinematic Guidance Model
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [TOUT3,YOUT3] = ode45(@(t,y) KinematicGuidanceModel(t,y,wind_inertial,control_objectives4,guidance_params),TSPAN,x_array3(:,i));

    x_array3(:,i+1) = YOUT3(end,:)';
    time_iter3(i+1) = TOUT3(end);
    wind_array3(:,i+1) = wind_inertial;
end


%Problem 3 Plotting:
figure()
hold on
plot3(pos_array3(1,:),pos_array3(2,:),-pos_array3(3,:))
plot3(n_pts_line,e_pts_line,h_pts_line,'r--')
title('Problem 3')
xlabel('North Position (m)')
ylabel('East Position (m)')
zlabel('Height (m)')
legend('Aicraft','Intended Path')


% ------------ PROBLEM 4 ------------ %
% problem 4:
% implement StraightLineGuidance with full aircraft eom, use loop from
% homework 7 with this function now

x_init4 = x_star;
x_init4(3) = -1805;
x_init4(4) = 0*pi/180;
aircraft_array4(:,1) = x_init4;
control_array4(:,1) = u_star;
time_iter3(1) = 0;
control_gain_struct.Ts=Ts;

for i=1:n_ind

    TSPAN = Ts*[i-1 i];

    wind_array4(:,i) = wind_inertial;

    wind_body4 = TransformFromInertialToBody(wind_inertial, aircraft_array4(4:6,i));
    air_rel_vel_body4 = aircraft_array4(7:9,i) - wind_body4;
    wind_angles4(:,i) = AirRelativeVelocityVectorToWindAngles(air_rel_vel_body4);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Guidance level commands
    %%%
    %%%   control_objectives(1) = h_c;
    %%%   control_objectives(2) = h_dot_c;
    %%%   control_objectives(3) = chi_c;
    %%%   control_objectives(4) = chi_dot_ff;
    %%%   control_objectives(5) = Va_c;
    %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % STUDENTS WRITE THIS FUNCTION
    control_objectives4 = StraightLineGuidance(aircraft_array4(1:3,i), line_speed,r,q,line_params); 


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autopilot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [control_out, x_c_out] = SLCWithFeedForwardAutopilot(Ts*(i-1), aircraft_array4(:,i), wind_angles4(:,i), control_objectives4, control_gain_struct);
    
    control_array4(:,i) = control_out;
    x_command4(:,i) = x_c_out;
    x_command4(5,i) = trim_variables(1);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Aircraft dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [TOUT2,YOUT2] = ode45(@(t,y) AircraftEOM(t,y,control_array4(:,i),wind_inertial,aircraft_parameters),TSPAN,aircraft_array4(:,i),[]);


    aircraft_array4(:,i+1) = YOUT2(end,:)';
    time_iter4(i+1) = TOUT2(end);
    wind_array4(:,i+1) = wind_inertial;
    control_array4(:,i+1) = control_array4(:,i);
    x_command4(:,i+1) = x_command4(:,i);
end
% PlotSimulationWithCommands(time_iter4,aircraft_array4,control_array4, wind_array4, x_command4, 'b')
% 
% figure(8)
% hold on
% plot3(n_pts_line,e_pts_line,h_pts_line,'r--')
% title('Problem 4')
% xlabel('North Position (m)')
% ylabel('East Position (m)')
% zlabel('Height (m)')



% ------------ PROBLEM 5 ------------ %
% problem 5: plot all together

% --- 1. Extract Data for Problem 2 (First Order Guidance) ---
% Preallocate arrays
chi_sim2 = zeros(length(t_sim2), 1);
Va_sim2  = zeros(length(t_sim2), 1);

for idx = 1:length(t_sim2)
    % Evaluate the derivative function to get velocity components [pn_dot; pe_dot; pd_dot]
    x_dot = FirstOrderLineGuidance(t_sim2(idx), y_sim2(idx,:)', line_speed, r, q, line_params);
    
    % Course angle from north and east velocities
    chi_sim2(idx) = atan2(x_dot(2), x_dot(1));
    
    % Airspeed (magnitude of velocity vector since wind is zero)
    Va_sim2(idx) = sqrt(x_dot(1)^2 + x_dot(2)^2 + x_dot(3)^2); 
end
h_sim2 = -y_sim2(:,3); % Height is negative Down position

% --- 2. Extract Data for Problem 3 (Kinematic Guidance Model) ---
% x_array3 = [pn; pe; chi; chi_dot; h; h_dot; Va]
Va_sim3  = x_array3(7, :);
chi_sim3 = x_array3(3, :);
h_sim3   = x_array3(5, :);

% --- 3. Extract Data for Problem 4 (Full Aircraft EOM) ---
% Preallocate flight angles array
flight_angles4 = zeros(3, length(time_iter4));

for i = 1:length(time_iter4)
    flight_angles4(:,i) = FlightPathAnglesFromState(aircraft_array4(:,i));
end

flight_angles4 = flight_angles4';
Va_sim4  = flight_angles4(:, 1);
chi_sim4 = flight_angles4(:, 2);
h_sim4   = -aircraft_array4(3, :)';

% --- 4. Plotting All Together ---
figure('Name', 'Problem 5: Guidance Model Comparison');

% Subplot 1: Airspeed
subplot(3, 1, 1)
hold on; grid on;
plot(t_sim2, Va_sim2, 'LineWidth', 1.5)
plot(time_iter3, Va_sim3, 'LineWidth', 1.5)
plot(time_iter4, Va_sim4, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Airspeed (m/s)')
legend('First Order Guidance', 'Kinematic Guidance', 'Full EOM', 'Location', 'best')
title('Aircraft State Comparison Across Guidance Models')

% Subplot 2: Course Angle
subplot(3, 1, 2)
hold on; grid on;
plot(t_sim2, chi_sim2, 'LineWidth', 1.5)
plot(time_iter3, chi_sim3, 'LineWidth', 1.5)
plot(time_iter4, chi_sim4, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Course Angle (rad)')
legend('First Order Guidance', 'Kinematic Guidance', 'Full EOM', 'Location', 'best')

% Subplot 3: Height
subplot(3, 1, 3)
hold on; grid on;
plot(t_sim2, h_sim2, 'LineWidth', 1.5)
plot(time_iter3, h_sim3, 'LineWidth', 1.5)
plot(time_iter4, h_sim4, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Height (m)')
legend('First Order Guidance', 'Kinematic Guidance', 'Full EOM', 'Location', 'best')