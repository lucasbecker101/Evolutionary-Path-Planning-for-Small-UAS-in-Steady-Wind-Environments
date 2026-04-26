clc; close all; clear;

%Author: Lucas Becker
%Date: 2/10/2026
%ASEN 5128 Homework 5 main file
%Simulating different linear aircraft modes

%loading aircraft paramaters
ttwistor;

%% Problem 1

%defining trim for problem 1
Va = 18;
gamma = 0;
h = 1800;
x_td = [Va, gamma, h];

%calling function to build state space
[A_lon,B_lon,A_lat,B_lat] = ss_level(x_td,aircraft_parameters);

%putting into state space format:
[sys_lon,sys_lat] = ss_to_sys(A_lon,B_lon,A_lat,B_lat);

%calculating trim state:
[x_star, u_star, ~] = calculate_trim_level(x_td,aircraft_parameters);

%extracting eigen values:
[vectors_lon,lambda_lon] = eig(A_lon);
[vectors_lat,lambda_lat] = eig(A_lat);

%extracting modes for longitudinal dynamics
idx_ph = 4;
idx_sp = 2; %manually extracting index of short period
vec_sp = vectors_lon(:,idx_sp);
vec_ph = vectors_lon(:,idx_ph);

%scaling eigen vectors to have 2 degree pitch angle perturbation
vec_sp = real((5 * pi / 180) * vec_sp / vec_sp(4));
vec_ph = real((15 * pi / 180) * vec_ph / vec_ph(4));

t_sim_ph = [0; 250];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Problem 1.1: simulating phugoid mode
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[y_ph_lin,t_ph_lin] = initial(sys_lon,vec_ph,t_sim_ph);
y_ph_lin = y_ph_lin';

%putting control inputs into array:
for i = 1:length(t_ph_lin)
    u_star_array_ph(:,i) = u_star;
end

%simulating with ode45 to provide entire time history of aircraft at trim
%with no perturbations, so linear perturbations can be added:
[t_star,y_star_ph] = ode45(@(t,x)AircraftEOM(t,x,u_star,[0; 0; 0] ,aircraft_parameters),t_ph_lin,x_star);

%adding linear perturbations to trim flight
y_ph_lin_fullstate = y_star_ph';
y_ph_lin_fullstate(7,:) = y_ph_lin_fullstate(7,:) + y_ph_lin(1,:);
y_ph_lin_fullstate(11,:) = y_ph_lin_fullstate(11,:) + y_ph_lin(3,:);
y_ph_lin_fullstate(5,:) = y_ph_lin_fullstate(5,:) + y_ph_lin(4,:);
y_ph_lin_fullstate(3,:) = y_ph_lin_fullstate(3,:) - y_ph_lin(5,:);


%generating non linear initial state:
x_star_nonlin_ph = x_star;
x_star_nonlin_ph(7) = x_star_nonlin_ph(7) + vec_ph(1);
x_star_nonlin_ph(11) = x_star_nonlin_ph(11) + vec_ph(3);
x_star_nonlin_ph(5) = x_star_nonlin_ph(5) + vec_ph(4);
x_star_nonlin_ph(3) = x_star_nonlin_ph(3) - vec_ph(5);

%simulating and plotting full non linear motion with phugoid mode excited
[t_nonlin_ph,y_nonlin_ph] = ode45(@(t,x)AircraftEOM(t,x,u_star,[0;0;0],aircraft_parameters),t_ph_lin,x_star_nonlin_ph);


%phugoid plotting
%PlotSimulation(t_ph_lin, y_ph_lin_fullstate', u_star_array_ph,'b')
%PlotSimulation(t_nonlin_ph,y_nonlin_ph,u_star_array_ph,'r')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PROBLEM 1.2 simulating short period mode
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_sim_sp1 = [0, 25];
t_sim_sp2 = [0 25];
%simulating with linear dynamics
[y_sp_lin,t_sp_lin] = initial(sys_lon,vec_sp,t_sim_sp1);
y_sp_lin = y_sp_lin';

[~,y_star_sp] = ode45(@(t,x)AircraftEOM(t,x,u_star,[0; 0; 0] ,aircraft_parameters),t_sp_lin,x_star);

%adding linear perturbations to trim flight
y_sp_lin_fullstate = y_star_sp';
y_sp_lin_fullstate(7,:) = y_sp_lin_fullstate(7,:) + y_sp_lin(1,:);
y_sp_lin_fullstate(11,:) = y_sp_lin_fullstate(11,:) + y_sp_lin(3,:);
y_sp_lin_fullstate(5,:) = y_sp_lin_fullstate(5,:) + y_sp_lin(4,:);
y_sp_lin_fullstate(3,:) = y_sp_lin_fullstate(3,:) - y_sp_lin(5,:);

%generating non linear initial state:
x_star_nonlin_sp = x_star;
x_star_nonlin_sp(7) = x_star_nonlin_sp(7) + vec_sp(1);
x_star_nonlin_sp(11) = x_star_nonlin_sp(11) + vec_sp(3);
x_star_nonlin_sp(5) = x_star_nonlin_sp(5) + vec_sp(4);
x_star_nonlin_sp(3) = x_star_nonlin_sp(3) - vec_sp(5);

%putting control inputs into array:
for i = 1:length(t_sp_lin)
    u_star_array_sp(:,i) = u_star;
end

%simulating and plotting full non linear motion with short period mode excited
[t_nonlin_sp,y_nonlin_sp] = ode45(@(t,x)AircraftEOM(t,x,u_star,[0;0;0],aircraft_parameters),t_sp_lin,x_star_nonlin_sp);

%short period plotting
%PlotSimulation(t_sp_lin, y_sp_lin_fullstate', u_star_array_sp,'b')
%PlotSimulation(t_nonlin_sp,y_nonlin_sp,u_star_array_sp,'r')


%% PROBLEM 2

%%% SAME TRIM AS PROBLEM 1 %%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Problem 2.1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_sim_21 = [0, 250];
pulse_array_21 = [1,0];     %defining pulse vs doublet
pulse_mag_21 = 5 * pi / 180;    %magnitude of pulse
pulse_idx_21 = 1;  %defining pulse on elevator


[t_21,y_21] = ode45(@(t,x)AircraftEOMPulsed(t,x,u_star,[0;0;0],aircraft_parameters,pulse_array_21,pulse_mag_21,pulse_idx_21),t_sim_21,x_star);

for i = 1:length(t_21)
    u_star_array_21(:,i) = u_star;

    if pulse_array_21(1) == 1

    if t_21(i) < 1
        del = pulse_mag_21;
    else
        del = 0;
    end

elseif pulse_array_21(2) == 1

%doublet:
    if t_21(i) < 1
        del = pulse_mag_21;
    elseif t_21(i) < 2  && t_21(i) >= 1
        del = -pulse_mag_21;
    end
end

u_star_array_21(pulse_idx_21) = u_star_array_21(pulse_idx_21) + del;
end


%PlotSimulation(t_21,y_21,u_star_array_21,'g')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Problem 2.2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_sim_22 = [0 100];
pulse_array_22 = [0,1];
pulse_mag_22 = 15 * pi / 180;
pulse_idx_22 = 2;

[t_22,y_22] = ode45(@(t,x)AircraftEOMPulsed(t,x,u_star,[0;0;0],aircraft_parameters, pulse_array_22, pulse_mag_22, pulse_idx_22), t_sim_22, x_star);


for i = 1:length(t_22)
    u_star_array_22(:,i) = u_star;

    if pulse_array_22(1) == 1

    if t_22(i) < 1
        del = pulse_mag_22;
    else
        del = 0;
    end

elseif pulse_array_22(2) == 1

%doublet:
    if t_22(i) < 0.5
        del = pulse_mag_22;
    elseif t_22(i) < 1  && t_22(i) >= 0.5
        del = -pulse_mag_22;
    elseif t_22(i) >= 1
        del = 0;
    end
end

u_star_array_22(pulse_idx_22,i) = u_star_array_22(pulse_idx_22,i) + del;
end


%PlotSimulation(t_22,y_22,u_star_array_22,'k')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Problem 2.3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_sim_23 = [0 150];
pulse_array_23 = [0,1];
pulse_mag_23 = 15 * pi / 180;
pulse_idx_23 = 3;

[t_23,y_23] = ode45(@(t,x)AircraftEOMPulsed(t,x,u_star,[0;0;0],aircraft_parameters, pulse_array_23, pulse_mag_23, pulse_idx_23), t_sim_23, x_star);


for i = 1:length(t_23)
    u_star_array_23(:,i) = u_star;

    if pulse_array_23(1) == 1

    if t_23(i) < 1
        del = pulse_mag_23;
    else
        del = 0;
    end

elseif pulse_array_23(2) == 1

%doublet:
    if t_23(i) < 0.5
        del = pulse_mag_23;
    elseif t_23(i) < 1  && t_23(i) >= 0.5
        del = -pulse_mag_23;
    elseif t_23(i) >= 1
        del = 0;
    end
end

u_star_array_23(pulse_idx_23,i) = u_star_array_23(pulse_idx_23,i) + del;
end


%PlotSimulation(t_23,y_23,u_star_array_23,'b')






