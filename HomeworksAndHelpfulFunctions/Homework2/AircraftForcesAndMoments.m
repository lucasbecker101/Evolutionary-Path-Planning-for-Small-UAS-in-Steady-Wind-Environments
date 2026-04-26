function [aircraft_forces, aircraft_moments] = AircraftForcesAndMoments(aircraft_state, aircraft_surfaces,wind_inertial, density, aircraft_parameters)
%Author: Lucas Becker
%Date: 1/20/2026
%Create the above function that takes as input the aircraft state, the control input vector,
% the inertial wind velocity in inertial coordinates, the air density, and the aircraft parameters
% structure and returns the total force and moment acting on the aircraft expressed in body
% coordinates. The total force includes the aerodynamic force, propulsive force, and weight.
% The total moment includes aerodynamic and propulsive moments. The output of the function
% should be two vectors, one for the force and one for the moment.

%calling function to provide aerodynamic forces and moments, moments dont
%change
[aero_force, aircraft_moments] = AeroForcesAndMoments_BodyState_WindCoeffs(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

%pulling out euler angles
phi = aircraft_state(4);
theta = aircraft_state(5);
psi = aircraft_state(6);

%calculating gravity forces in each direction
force_grav = [-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)] * aircraft_parameters.g * aircraft_parameters.m;

%combining aero forces with gravity force
aircraft_forces = aero_force + force_grav;




end