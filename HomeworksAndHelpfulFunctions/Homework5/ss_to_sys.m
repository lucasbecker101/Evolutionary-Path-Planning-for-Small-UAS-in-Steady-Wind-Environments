function [sys_lon, sys_lat] = ss_to_sys(A_lon,B_lon,A_lat,B_lat)
%Author: Lucas Becker
%Date: 2/10/2026
%ASEN 5128 Homework 5 function
%creating matlab sys variables for given longitudinal and lateral state
%space matrices

sys_lon = ss(A_lon,B_lon, eye(size(A_lon,1),size(A_lon,1)), zeros(size(A_lon,1),size(B_lon,2)));
sys_lat = ss(A_lat,B_lat, eye(size(A_lat,1),size(A_lat,1)), zeros(size(A_lat,1),size(B_lat,2)));




end