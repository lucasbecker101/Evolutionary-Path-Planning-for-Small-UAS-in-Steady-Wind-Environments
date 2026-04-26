function [modes] = linear_aircraft_modes(A_lon, A_lat)
%Author: Lucas Becker
%Date: 2/2/2026
%ASEN 5128 Homework4 linear aircraft modes
%pulls out eigen vectors and values for longitudinal and lateral state
%spaces, sorts them into the different aircraft modes, and 


%Extracting eigen values and vectors from both A matrices
[lon_vects, lon_values] = eig(A_lon);
[lat_vects, lat_values] = eig(A_lat);


%sorting longitudinal eigen vectors
for i = 1:length(lon_values)
    
    omega_n(1) = 

end

end