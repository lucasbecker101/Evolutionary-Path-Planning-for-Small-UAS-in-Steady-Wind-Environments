clc; close all; clear;

%%ASEN 5128 HOMEWORK 3
%Author: Lucas Becker
%Date: 1/26/2026

%load aircraft parameters
ttwistor

%set common time span
tspan = [0,300];


%% Problem 3.1

%trim definition from problem statement
V1 = 18;
h1 = 1655;
gamma1 = 0;
x_td1 = [V1;gamma1;h1];

%wind condition:
wind1 = [0;0;0];

%calculating initial trim state
[x_trim1,u_trim1,fval1] = calculate_trim_level(x_td1,aircraft_parameters);

%simulate with ode45 and aircraftEOM function:
[t1,y1] = ode45(@(t,x)AircraftEOM(t,x,u_trim1,wind1,aircraft_parameters),tspan,x_trim1);

%putting control inputs into array:
for i = 1:length(t1)
    u_1(:,i) = u_trim1;
end

%plotting:
%PlotSimulation(t1,y1,u_1,'c')



%% Problem 3.2
%trim definition given in problem statement
x_td2 = x_td1;
wind2 = [10;10;0];

%calculating trim state:
[x_trim2, u_trim2, fval2] = calculate_trim_level(x_td2, aircraft_parameters);

%adding in wind:
wind_body = TransformFromInertialToBody(wind2,x_trim2(4:6));
x_trim2(7:9) = wind_body + x_trim2(7:9);

%Simulating with ode45:
[t2,y2] = ode45(@(t,x)AircraftEOM(t,x,u_trim2,wind2,aircraft_parameters),tspan,x_trim2);

%putting control inputs into array:
for i = 1:length(t2)
    u_2(:,i) = u_trim2;
end

%plotting simulation 2:
%PlotSimulation(t2,y2,u_2,'r--')

%% Problem 3.3

%trim definition
gamma3 = 10*pi/180;
x_td3 = [V1,gamma3,h1];
wind3 = [0;0;0];

%calculating trim state for problem 3.3:
[x_trim3, u_trim3, fval3] = calculate_trim_level(x_td3, aircraft_parameters);

% Simulating with ode45 for problem 3.3:
[t3, y3] = ode45(@(t, x) AircraftEOM(t, x, u_trim3, wind3, aircraft_parameters), tspan, x_trim3);

%putting control inputs into array for problem 3.3:
for i = 1:length(t3)
    u_3(:,i) = u_trim3;
end

%plotting simulation 3:
%PlotSimulation(t3, y3, u_3, 'g--')


%% Problem 3.4: Coordinated Turn

%trim definition
h4 = 200;
Va4 = 20.2;
R4 = 500;
gamma4 = 0;
wind4 = [0;0;0];

x_td4 = [Va4, gamma4, h4, R4];

%calculating trim state for problem 3.4:
[x_trim4, u_trim4, fval4] = calculate_trim_turn(x_td4, aircraft_parameters);


% Simulating with ode45 for problem 3.4:
[t4, y4] = ode45(@(t, x) AircraftEOM(t, x, u_trim4, wind4, aircraft_parameters), tspan, x_trim4);

%putting control inputs into array for problem 3.4:
for i = 1:length(t4)
    u_4(:,i) = u_trim4;
end

%plotting simulation 4:
%PlotSimulation(t4, y4, u_4, 'm')



%PlotSimulation(t1,y1,u_1,'c')
%PlotSimulation(t2, y2, u_2, 'r')
%PlotSimulation(t3, y3, u_3, 'g')
PlotSimulation(t4, y4, u_4, 'm')