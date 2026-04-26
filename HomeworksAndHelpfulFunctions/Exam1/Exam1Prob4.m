clc; clear;
%close all;  %comment to plot on the same figure
% Title: ASEN 5128 Exam 1 code for problem 4
% Author: Lucas Becker
% Date Created: 02/28/2026
%-------EXAM 1 Problem 4-------

%adding path to access all functions made for this class
addpath(genpath('C:\Users\lucas\OneDrive\Documents\Current Classes\sUAS_GNC\Code'))

%retrieving aircraft parameters for raaven
aircraft = 0;   %set to one for raaven, anything else for ttwistor
if aircraft == 1
    raaven
    col = 'b';
else
    ttwistor
    col = 'r';
end

%determining trim condition:
Va_star = 20;
gamma_star = 0;
h_star = 1800;
x_td = [Va_star;gamma_star;h_star];
h0 = 1655;
Xc = 60 * pi / 180;

%calculating trim state and variables
[x_star,u_star,x_tv,~] =  calculate_trim_level(x_td, aircraft_parameters);

load('raaven_gains_slc.mat')

%Following code for simulation taken from RunHW7 file originally created
%by Eric Frew

%set initial state and conditions:
x0 = x_star;
x0(3) = -h0;
u0 = u_star;
wind_inertial = [0;0;0];


% Set simulation and control parameters
Ts = .1;
Tfinal = 200;
control_gain_struct.Ts=Ts;


%%% iterate at control sample time
n_ind = Tfinal/Ts;

%initializing simulation arrays
aircraft_array(:,1) = x0;
control_array(:,1) = u0;
time_iter(1) = 0;

%setting commanded inputs
h_c         = h_star;  % commanded altitude (m)
h_dot_c     = 0;  % commanded altitude rate (m)
chi_c       = 60*pi/180;  % commanded course (rad)
chi_dot_ff  = 0;  % commanded course rate (rad)   
Va_c        = Va_star;  % commanded airspeed (m/s)


%simulation loop:
for i=1:n_ind

    TSPAN = Ts*[i-1 i];

    wind_array(:,i) = wind_inertial;

    wind_body = TransformFromInertialToBody(wind_inertial, aircraft_array(4:6,i));
    air_rel_vel_body = aircraft_array(7:9,i) - wind_body;
    wind_angles(:,i) = AirRelativeVelocityVectorToWindAngles(air_rel_vel_body);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Guidance level commands
    %%%
    %%% Note, the format is to allow flexibility for future assignments
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    control_objectives(1) = h_c;
    control_objectives(2) = h_dot_c;
    control_objectives(3) = chi_c;
    control_objectives(4) = chi_dot_ff;
    control_objectives(5) = Va_c;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autopilot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [control_slc, x_c_slc] = SimpleSLCAutopilot(Ts*(i-1), aircraft_array(:,i), wind_angles(:,i), control_objectives, control_gain_struct);

    control_array(:,i) = control_slc;
    x_command(:,i) = x_c_slc;
    x_command(5,i) = x_tv(1);


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



PlotSimulationWithCommands(time_iter,aircraft_array,control_array, wind_array, x_command, col)





