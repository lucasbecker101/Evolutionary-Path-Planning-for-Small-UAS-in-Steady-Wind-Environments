clc; close all; clear;

%Author: Lucas Becker
%Date: 2/2/2026
%ASEN 5128 Homework4 main file
%calling and testing files to calculate state space for a system given trim
%definition for straight, level flight and aircraft parameters. Then
%determining aircraft modes and characteristics of those modes.


%loading aircraft paramaters
ttwistor;

%Defining trim definition
Va = 18;
gamma = 0;
h_trim = 1800;
x_td = [Va, gamma, h_trim];

%calling function to build state space
[A_lon,B_lon,A_lat,B_lat] = ss_level(x_td,aircraft_parameters);


%compute eigen values and vectors for longitudinal and lateral
[lon_vectors,lon_values] = eig(A_lon);
[lat_vectors,lat_values] = eig(A_lat); 

%Outputting table with poles, damping, and natural frequency for all eigen
%values

%formulating system for longitudinal mode and usimg damp command to find
%systems modes and characteristics
sys_lon = ss(A_lon,B_lon, zeros(size(A_lon,1),size(A_lon,1)), zeros(size(A_lon,1),size(B_lon,2)));
damp(sys_lon)
%figure(1)
%pzmap(sys_lon) %plots poles

%formulating system for longitudinal mode and usimg damp command to find
%systems modes and characteristics
sys_lat = ss(A_lat,B_lat, zeros(size(A_lat,1),size(A_lat,1)), zeros(size(A_lat,1),size(B_lat,2)));
damp(sys_lat)
%figure(2)
%pzmap(sys_lat,'r') %plots poles



%% Test code
x_td_test = [20, 0, 200];

% Calling function to build state space for the test case
[A_lon_test, B_lon_test, A_lat_test, B_lat_test] = ss_level(x_td_test, aircraft_parameters);
ss_test_lat = ss(A_lat_test,B_lat_test, zeros(size(A_lat_test,1),size(A_lat_test,1)), zeros(size(A_lat_test,1),size(B_lat_test,2)));
ss_test_lon = ss(A_lon_test,B_lon_test, zeros(size(A_lon_test,1),size(A_lon_test,1)), zeros(size(A_lon_test,1),size(B_lon_test,2)));

