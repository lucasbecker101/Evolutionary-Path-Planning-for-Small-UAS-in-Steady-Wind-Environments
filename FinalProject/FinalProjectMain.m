  % Author: Lucas Becker
% Date Created: 3/24/2026
% Most Recent Edit: 4/25/2026
% Title: ASEN 5128 Final Project (Main Script)
% Description: The purpose of this script is to simulate and compare
% evolutionary path planning algorithms developed for the RAAVEN Aircraft.
% The environment will be set up as a series of static obstacles set in a
% constant wind field. The evolutionary algorthm will attempt to find the
% near optimal path to an environment

close all;% <========= Comment out this line and you can run this file multiple times and plot results together
clear
clc

addpath(genpath('C:\Users\lucas\OneDrive\Documents\Current Classes\sUAS_GNC\Code'))

% ------------ Loading Aircraft Parameters -------------
% Aircraft parameters
ttwistor

% Determine trim state and control inputs
g = aircraft_parameters.g;
V_trim = 18;
h_trim = 1805;
gamma_trim = 0;
trim_definition = [V_trim; gamma_trim; h_trim];
[xstar,ustar,trim_variables,fval] =  calculate_trim_level(trim_definition, aircraft_parameters);

% Load control gains
gains_file = 'ttwistor_gains_feed';
fprintf(1, '\n==================================== \nAUTOPILOT: SLC with Feedforward\n \n')
load(gains_file)

% -------------- Generating Environment ----------------
% ENVIRONMENT PARAMETERS
env_size = 5000;        % meters
n_obs = 15;             % 15 obstacles
max_w = 10;             % Max wind of 10 m/s
seed = 5;               % Set seed to get same environment every time

% Define two different resolutions
disc = 10;              % FINE Discretization for accuracy
plot_disc = 100;         % COARSE Discretization for plotting

% Aircraft Start and Target Locations:
aircraft_start = [0;0];
aircraft_target = [env_size; env_size]; 

% % Call the function (Using the fine discretization)
[wind, obs] = get_environment(env_size, n_obs, max_w, disc, seed);
% 
% Plotting Environment
figure()
hold on

% Create physical coordinates for the fine computational cells
grid_coords = (disc/2) : disc : (env_size - (disc/2));

% Plot obstacles using the fine physical coordinate vectors
imagesc(grid_coords, grid_coords, obs) 
colormap(flipud(gray)) % Make obstacles dark, empty space white
axis equal tight

% Set axis limits exactly to the environment bounds
xlim([0, env_size])
ylim([0, env_size])

% Flip the Y-axis so 0 is at the bottom (standard Cartesian)
set(gca, 'YDir', 'normal')

% --- Subsample Wind for Plotting ---
% Calculate how many indices to skip to achieve the plotting resolution
stride = round(plot_disc / disc); 

% Subsample the coordinates and the wind matrices using the stride
plot_coords = grid_coords(1:stride:end);
[X_plot, Y_plot] = meshgrid(plot_coords, plot_coords);

wind_x_plot = wind(1:stride:end, 1:stride:end, 1);
wind_y_plot = wind(1:stride:end, 1:stride:end, 2);

% Plot the subsampled wind vectors
quiver(X_plot, Y_plot, wind_x_plot, wind_y_plot, 0.75, 'r') 

% Plot Start Location (Bright green filled circle)
plot(aircraft_start(1), aircraft_start(2), 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

% Plot Target Location (Bright red filled star)
plot(aircraft_target(1), aircraft_target(2), 'p', 'MarkerSize', 16, ...
    'MarkerFaceColor', [1 0.2 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

title('UAS Environment: Wind Vectors and Obstacles')
xlabel('X Position (m)')
ylabel('Y Position (m)')



% --------- Running Evolutionary Algorithm ------------

% Initialize population of random trajectories
population_size = 40; % Number of paths
num_waypoints = 8; % Number of waypoints per path
iterations = 100;   % number of iterations path planning is allowed
phi_max = deg2rad(45);  %setting max bank angle of 45 degrees to determine minimum radius of a turn
R_min = V_trim^2 / (g * tan(phi_max));

throttle_cost = 1;
no_throttle_cost = 0;


% for throttle cost
[best_path1, best_score1, best_time_score1, best_throttle_score1] = EvolutionaryPathPlanning(population_size, num_waypoints, iterations, R_min, wind, obs, env_size, aircraft_parameters, control_gain_struct, xstar, ustar, throttle_cost);

% no throttle cost
[best_path2, best_score2, best_time_score2, best_throttle_score2] = EvolutionaryPathPlanning(population_size, num_waypoints, iterations, R_min, wind, obs, env_size, aircraft_parameters, control_gain_struct, xstar, ustar, no_throttle_cost);




% =========================================================================
% PLOT: COMPARISON OF BEST PATHS
% =========================================================================
figure('Name', 'Best Paths Comparison', 'NumberTitle', 'off');
hold on;

% --- 1. Plot Obstacles ---
imagesc(grid_coords, grid_coords, obs);
colormap(flipud(gray)); % Make obstacles dark, empty space white
axis equal tight;
xlim([0, env_size]);
ylim([0, env_size]);
set(gca, 'YDir', 'normal'); % Flip the Y-axis so 0 is at the bottom

% --- 2. Plot Wind ---
% Re-use the subsampled wind coordinates you already calculated
quiver(X_plot, Y_plot, wind_x_plot, wind_y_plot, 0.75, 'r', 'DisplayName', 'Wind');

% --- 3. Plot Best Path 1 (With Throttle Cost) ---
% Plot the dense line in Blue
plot(best_path1.points(:,1), best_path1.points(:,2), '-', 'Color', [0.2 0.4 0.8], ...
     'LineWidth', 2.5, 'DisplayName', 'Path 1 (Throttle Cost)');
% Plot the waypoints (hidden from legend for cleanliness)
plot(best_path1.waypoints(:,1), best_path1.waypoints(:,2), 'o', 'Color', [0.2 0.4 0.8], ...
     'MarkerFaceColor', [0.2 0.4 0.8], 'MarkerSize', 6, 'HandleVisibility', 'off');

% --- 4. Plot Best Path 2 (No Throttle Cost) ---
% Plot the dense line in Orange
plot(best_path2.points(:,1), best_path2.points(:,2), '-', 'Color', [0.9 0.5 0.1], ...
     'LineWidth', 2.5, 'DisplayName', 'Path 2 (No Throttle Cost)');
% Plot the waypoints
plot(best_path2.waypoints(:,1), best_path2.waypoints(:,2), 's', 'Color', [0.9 0.5 0.1], ...
     'MarkerFaceColor', [0.9 0.5 0.1], 'MarkerSize', 6, 'HandleVisibility', 'off');

% --- 5. Plot Start and Target Locations ---
plot(aircraft_start(1), aircraft_start(2), 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
    'DisplayName', 'Start');
plot(aircraft_target(1), aircraft_target(2), 'p', 'MarkerSize', 16, ...
    'MarkerFaceColor', [1 0.2 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
    'DisplayName', 'Target');

% --- 6. Formatting ---
title('Comparison of Best Paths: Throttle Cost vs. No Throttle Cost');
xlabel('X Position (m)');
ylabel('Y Position (m)');

% Create a legend, but ignore the 'imagesc' layer so it doesn't show a weird color box
ax = gca;
children = ax.Children;
% Exclude the Image object (which is the last object added to ax.Children)
legend(children(1:end-1), 'Location', 'best'); 

hold off;
