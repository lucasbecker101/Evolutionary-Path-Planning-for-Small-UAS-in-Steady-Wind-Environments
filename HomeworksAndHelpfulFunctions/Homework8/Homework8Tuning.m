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
%load("ttwistor_gains_slc");

SLC = 2;
FEED = 1;
CONTROL_FLAG = FEED;

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
tsim1 = t0:0.1:tf;
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

%setting objectives and offsets to help tune:
h_tuning = 100;
chi_tuning = 30 * pi / 180 * 1;
Va_tuning = 1 * 1; 
objectives1 = [h_trim; 0; chi_tuning; 0; V_trim + Va_tuning];

h_init = 1655;
xinit1 = [0;0;0;0;h_init;0;V_trim];
%running problem 1 simulation
[tsim1,ysim1] = ode45(@(t,x) KinematicGuidanceModel(t,x,wind_inertial,objectives1,guidance_params),tsim1,xinit1);





%simulating full aircraft eom for comparison
Ts = .1;
Tfinal = 200;
control_gain_struct.Ts=Ts;

%%% iterate at control sample time
n_ind = Tfinal/Ts;

aircraft_state0 = x_star;

aircraft_state0(3,1) = -h_init; %<------- CLIMB mode starts when aircraft reaches h = 1675
aircraft_state0(4,1) = 0*pi/180;

control_input0 = u_star;
aircraft_array(:,1) = aircraft_state0;
control_array(:,1) = control_input0;
time_iter(1) = 0;

for i=1:n_ind

    TSPAN = Ts*[i-1 i];

    wind_array(:,i) = wind_inertial;

    wind_body = TransformFromInertialToBody(wind_inertial, aircraft_array(4:6,i));
    air_rel_vel_body = aircraft_array(7:9,i) - wind_body;
    wind_angles(:,i) = AirRelativeVelocityVectorToWindAngles(air_rel_vel_body);


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
    %control_objectives = OrbitGuidance(aircraft_array(1:3,i), orbit_speed, orbit_radius, orbit_center, orbit_flag, orbit_gains); 
    % control_gain_struct.Kp_course_rate=0; %<============== Uncomment if your guidance algorithm does not give a command course angle, i.e. only gives commanded course rate
    % control_gain_struct.Kff_course_rate = 0; %<============== Uncomment if your guidance algorithm does not give a command course rate

    control_objectives = [h_trim; 0; chi_tuning; 0; V_trim + Va_tuning]; %<============== Comment out when OrbitGuidance is complet
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autopilot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if (CONTROL_FLAG==FEED)
        [control_out, x_c_out] = SLCWithFeedForwardAutopilot(Ts*(i-1), aircraft_array(:,i), wind_angles(:,i), control_objectives, control_gain_struct);
    else
        [control_out, x_c_out] = SimpleSLCAutopilot(Ts*(i-1), aircraft_array(:,i), wind_angles(:,i), control_objectives, control_gain_struct);
    end
    
    control_array(:,i) = control_out;
    x_command(:,i) = x_c_out;
    x_command(5,i) = trim_variables(1);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Aircraft dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [TOUT2,YOUT2] = ode45(@(t,y) AircraftEOM(t,y,control_array(:,i),wind_inertial,aircraft_parameters),TSPAN,aircraft_array(:,i),[]);


    aircraft_array(:,i+1) = YOUT2(end,:)';
    time_iter(i+1) = TOUT2(end);
    wind_array(:,i+1) = wind_inertial;
    control_array(:,i+1) = control_array(:,i);
    x_command(:,i+1) = x_command(:,i);
end

for i = 1:length(time_iter)
    flight_angles(:,i) = FlightPathAnglesFromState(aircraft_array(:,i));
end
flight_angles = flight_angles';

figure()

subplot(311)
hold on
plot(tsim1,ysim1(:,7))
plot(time_iter, flight_angles(:,1))
xlabel('Time (s)')
ylabel('Airspeed (m/s)')
legend('Kinematic Guidance','Full EOM')

subplot(312)
hold on
plot(tsim1,ysim1(:,3))
plot(time_iter, flight_angles(:,2))
xlabel('Time (s)')
ylabel('Course Angle (rad)')
legend('Kinematic Guidance','Full EOM')

subplot(313)
hold on
plot(tsim1,ysim1(:,5))
plot(time_iter,-aircraft_array(3,:))
xlabel('Time (s)')
ylabel('Height (m)')
legend('Kinematic Guidance','Full EOM')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%PlotSimulation(time_iter,aircraft_array,control_array, wind_array,'b')
%PlotSimulationWithCommands(time_iter,aircraft_array,control_array, wind_array, x_command, col)