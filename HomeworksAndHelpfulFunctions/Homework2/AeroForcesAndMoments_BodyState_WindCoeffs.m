function [aero_force, aero_moment] = AeroForcesAndMoments_BodyState_WindCoeffs(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters)
%Author: Lucas Becker
%Date: 1/20/2026
%Create the above function that takes as input the aircraft state, the control input vector,
% the inertial wind velocity in inertial coordinates, the air density, and the aircraft parameters
% structure and returns the aerodynamic force and moment acting on the aircraft expressed in
% body coordinates. For this function the propulsive force is considered part of the aerodynamic
% force, and it DOES NOT include weight. The moment includes aerodynamic and propulsive
% moments. The output of the function should be two vectors, one for the force and one for
% the moment.

%THRUST INCLUDED IN AERODYNAMIC FORCES

R_inertial_to_body = RotationMatrix321(aircraft_state(4:6));
wind_body = R_inertial_to_body * wind_inertial;

velocity_air = aircraft_state(7:9) - wind_body;

%wind angles:   
wind_angles = AirRelativeVelocityVectorToWindAngles(velocity_air);
V = wind_angles(1);
beta = wind_angles(2);
alpha = wind_angles(3);



%dynamic pressure:
Q = 0.5 * density * V^2;


%non dimensional rates:
p_hat = (aircraft_state(10)*aircraft_parameters.b) / (2*V);
q_hat = (aircraft_state(11)*aircraft_parameters.c) / (2*V); 
r_hat = (aircraft_state(12)*aircraft_parameters.b) / (2*V);


%determining aerodynamics coefficients:
CL = aircraft_parameters.CL0 + aircraft_parameters.CLalpha * alpha + aircraft_parameters.CLq * q_hat + aircraft_parameters.CLde * aircraft_surfaces(1);%lift coefficient
CD = aircraft_parameters.CDmin + aircraft_parameters.K*(CL - aircraft_parameters.CLmin)^2;%drag coefficient
CT = 2 * (aircraft_parameters.Sprop/aircraft_parameters.S) * aircraft_parameters.Cprop * (aircraft_surfaces(4)/V^2) * (V + aircraft_surfaces(4) * (aircraft_parameters.kmotor - V))*(aircraft_parameters.kmotor - V);%Thrust coefficient
CY = aircraft_parameters.CYbeta * beta + aircraft_parameters.CYp * p_hat + aircraft_parameters.CYda * aircraft_surfaces(2) + aircraft_parameters.CYdr * aircraft_surfaces(3) + aircraft_parameters.CYr * r_hat;
Cl = aircraft_parameters.Clbeta * beta + aircraft_parameters.Clp * p_hat + aircraft_parameters.Clr * r_hat + aircraft_parameters.Clda * aircraft_surfaces(2) + aircraft_parameters.Cldr * aircraft_surfaces(3);
Cm = aircraft_parameters.Cm0 + aircraft_parameters.Cmalpha * alpha + aircraft_parameters.Cmq * q_hat + aircraft_parameters.Cmde * aircraft_surfaces(1);
Cn = aircraft_parameters.Cnbeta * beta + aircraft_parameters.Cnp * p_hat + aircraft_parameters.Cnr * r_hat + aircraft_parameters.Cnda * aircraft_surfaces(2) + aircraft_parameters.Cndr * aircraft_surfaces(3);

CX = -CD * cos(alpha) + CL*sin(alpha) + CT;
CZ = -CD * sin(alpha) - CL*cos(alpha);


%Determining aerodynamic forces
X = aircraft_parameters.S * Q * CX;
Y = aircraft_parameters.S * Q * CY;
Z = aircraft_parameters.S * Q * CZ;
L = aircraft_parameters.b * aircraft_parameters.S * Q * Cl;
M = aircraft_parameters.c * aircraft_parameters.S * Q * Cm;
N = aircraft_parameters.b * aircraft_parameters.S * Q * Cn;

%Put outputs into vectors
aero_force = [X; Y; Z];
aero_moment = [L; M; N];




end