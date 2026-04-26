% Lucas Becker
% ASEN 5128
% Homework9Main.m
% Edited: 3/25/2026
% plotting raaven wind data for asen 5128 homework 9
clc; clear;
close all;  %comment out for plotting on same figure

%adding all functions created for 5128 to path
addpath(genpath('C:\Users\lucas\OneDrive\Documents\Current Classes\sUAS_GNC\Code'))

%initializing RAAVEN aircraft parameters
raaven


% -------- Problem 1A ---------%
%defining given state and variables
f_grav_E = [0;0;9.807];
orientation = deg2rad([6;9;-75]);
omega = deg2rad([0.2;-1.4;4.8]);
omega_tilde = [0, -omega(3), omega(2);
               omega(3), 0, -omega(1);
               -omega(2), omega(1), 0];
v_B = [21;-1;4];

%transform gravity acceleration to body frame
f_grav_B = TransformFromInertialToBody(f_grav_E,orientation);

%calculate the body forces given the measured acceleration and known
%gravity
y_accel = [9.75;1.62;0.67];
body_forces = y_accel + f_grav_B;

%compute the chanage in each velocity
v_dot = -omega_tilde * v_B + body_forces;



% --------- Problem 1B ---------%
wind_body_measured = [12.2;-0.2;2.3];
wind_inertial_truth = [0;6;0];
wind_body_truth = TransformFromInertialToBody(wind_inertial_truth,orientation);




% --------- Problem 2 --------%
%loading RaavenWindData
load('RaavenWindData.mat')

euler_angles = deg2rad([roll'; pitch'; yaw']);
alpha = deg2rad(alpha);
beta = deg2rad(beta);


wind_inertial = SensorsToWind(aircraft_velocity_inertial, euler_angles, Va, beta, alpha, Time);


figure('Color', 'w');
hold on;

% Plot the colored trajectory
z = zeros(size(lon)); % Z-coordinates are zero (flat 2D plot)
surface([lon, lon], [lat, lat], [z, z], [Time, Time], ...
    'FaceColor', 'none', ...
    'EdgeColor', 'interp', ...
    'LineWidth', 3);

% Plot the wind vectors using 'quiver'
% Downsample the arrows so they don't overlap into a solid block
skip = 150; 
plot_points = 1:skip:length(lon);
quiver(lon(plot_points)', lat(plot_points)', wind_inertial(1,plot_points), wind_inertial(2,plot_points), 2);

% Formatting and styling
colormap('spring');
c = colorbar;
ylabel(c, 'Flight Time (minutes)', 'FontSize', 11);
xlabel('Longitude', 'FontSize', 11);
ylabel('Latitude', 'FontSize', 11);
title('RAAVEN Flight With Inertial Wind Overlayed')

% Grid styling
grid on;
set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.4);
box on;
