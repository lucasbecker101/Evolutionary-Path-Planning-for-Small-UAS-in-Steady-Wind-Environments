clc; close all; clear;

%Author: Lucas Becker
%Date: 1/20/2026
%ASEN5128 Homework 2
%Simulating Recuv's ttwistor aircraft for three different cases

%aircraft parameters
ttwistor;

%simulation parameters:
t_span = [0,200];


%% Question 3.1:
%defining initial state
h = 1655;
wind_1 = [0;0;0];
V_a1 = 18;

%determining initial velocity for state vector using wind angles
wind_angles_1 = [V_a1,0,0];
velocity_body_1 = WindAnglesToAirRelativeVelocityVector(wind_angles_1);

%defining initial state and control inputs
x_init_1 = [0;0;-h;0;0;0;velocity_body_1; 0;0;0];
u_1_vector = zeros(4,1);

%running simulation
[t1,y1] = ode45(@(t,x) AircraftEOM(t,x,u_1_vector,wind_1,aircraft_parameters),t_span,x_init_1);

%put control input into matrix form for plotting
for i = 1:length(t1)
    u_1(:,i) = u_1_vector;
end

%plot simulation 1
PlotSimulation(t1,y1,u_1,'r')


%% Question 3.2:
%given conditions
wind_2 = [10;10;0];
x_init_2 = x_init_1;
u_2_vector = u_1_vector;

%running second simulation
[t2,y2] = ode45(@(t,x) AircraftEOM(t,x,u_2_vector,wind_2,aircraft_parameters),t_span,x_init_2);


%put control input into matrix form for plotting
for i = 1:length(t2)
    u_2(:,i) = u_2_vector;
end

%plot simulation 2
PlotSimulation(t2,y2,u_2,'g')


%% Question 3.3:
%given conditions
x_init_3 = [0; 0; -1800; 15*pi/180; -12*pi/180; 270*pi/180; 19; 3; -2; 0.08*pi/180; -0.2*pi/180; 0];
u_3_vector = [5*pi/180; 2*pi/180; -13*pi/180; 0.3];
wind_3 = zeros(3,1);

%run simulation 3
[t3,y3] = ode45(@(t,x) AircraftEOM(t,x,u_3_vector,wind_3,aircraft_parameters),t_span,x_init_3);

%put control input into matrix form for plotting
for i = 1:length(t3)
    u_3(:,i) = u_3_vector;
end

%plot simulation 3
PlotSimulation(t3,y3,u_3,'b')



%% TEST:
x_test = [1;2;3;4;5;6;7;8;9;10;11;12];
u_test = [1;2;3;4];
wind_test = [1;2;3];
density = 1.1;

[test_aero_forces, test_aero_moments] = AeroForcesAndMoments_BodyState_WindCoeffs(x_test, u_test, wind_test, density, aircraft_parameters);
[test_aircraft_forces, test_aircraft_moments] = AircraftForcesAndMoments(x_test, u_test,wind_test, density, aircraft_parameters);
x_dot_test = AircraftEOM(t1,x_test,u_test,wind_test,aircraft_parameters);