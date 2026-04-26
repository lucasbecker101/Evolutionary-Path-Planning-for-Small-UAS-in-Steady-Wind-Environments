clc; close all; clear;
% Title: ASEN 5128 Exam 1 code for problem 2
% Author: Lucas Becker
% Date Created: 02/28/2026
%-------EXAM 1 Problem 2-------

%adding path to access all functions made for this class
addpath(genpath('C:\Users\lucas\OneDrive\Documents\Current Classes\sUAS_GNC\Code'))

%retrieving aircraft parameters for raaven
raaven

%given trim definition:
Va = 21; 
gamma = 0;
h = 1500;
x_td = [Va;gamma;h];

%---------Problem 2.1----------
%calculating trim variables and trim state for level flight:
[x_star,u_star, x_tv, ~] = calculate_trim_level(x_td,aircraft_parameters);

alpha_star = x_tv(1);
de_star = u_star(1);
dt_star = u_star(2);

%--------Problem 2.2--------
%linearizing dynamics and putting into A and B matrices
[A_lon, B_lon, A_lat, B_lat] = ss_level(x_td, aircraft_parameters);  

%formulating system for longitudinal dynamics and usimg damp command to find
%systems modes and characteristics
sys_lon = ss(A_lon,B_lon, zeros(size(A_lon,1),size(A_lon,1)), zeros(size(A_lon,1),size(B_lon,2)));
damp(sys_lon);


%formulating system for lateral dynamics and usimg damp command to find
%systems modes and characteristics
sys_lat = ss(A_lat,B_lat, zeros(size(A_lat,1),size(A_lat,1)), zeros(size(A_lat,1),size(B_lat,2)));
damp(sys_lat)

