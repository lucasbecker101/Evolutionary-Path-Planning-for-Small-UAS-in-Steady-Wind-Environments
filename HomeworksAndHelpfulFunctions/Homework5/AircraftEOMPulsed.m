function xdot = AircraftEOMPulsed(time,aircraft_state,aircraft_surfaces,wind_inertial,aircraft_parameters,pulse_array, pulse_mag, pulse_idx)
%Author: Lucas Becker
%Date: 1/20/2026
%Implement the full equations of motion by returning the derivative
% of the state vector, ˙x. I strongly recommend writing this function with vector equations,
% not by typing out the differential equation for each separate term. Note, the input time is
% needed for the Matlab simulation tools. Although it is not used, it needs to be included.

%calculate density based on height
[density,~,~,~,~] = stdatmo(-aircraft_state(3));

%PULSE ADDITIONS FOR HOMEWORK5

del = 0;

u_pulse = [aircraft_surfaces(1);0;0;aircraft_surfaces(4)];

if pulse_array(1) == 1 && time < 1
    del = pulse_mag;

elseif pulse_array(2) == 1
    if time < 0.5
        del = pulse_mag;
    elseif time < 1
        del = -pulse_mag;
    end
end

u_pulse(pulse_idx) = aircraft_surfaces(pulse_idx) + del;
 
 






%calling function to determine total aircraft forces and moments
[aircraft_forces, aircraft_moments] = AircraftForcesAndMoments(aircraft_state, u_pulse, wind_inertial, density, aircraft_parameters);

%extracting four sets of three from aircraft state to make equation
%formation easier
p = aircraft_state(1:3);
o = aircraft_state(4:6);
V = aircraft_state(7:9);
omega = aircraft_state(10:12);

%rotation matrix
R_body_to_earth = transpose(RotationMatrix321(o));

%transition matrix
T = [1, sin(o(1))*tan(o(2)), cos(o(1))*tan(o(2));
     0, cos(o(1)), -sin(o(1));
     0, sin(o(1))*sec(o(2)), cos(o(1))*sec(o(2))];

%omega_tilde
omega_tilde = [0, -omega(3), omega(2);
               omega(3), 0, -omega(1);
               -omega(2), omega(1), 0];


%calculating rate of change
p_dot = R_body_to_earth * V;
o_dot = T * omega;
V_dot = -omega_tilde * V + aircraft_forces/aircraft_parameters.m;
omega_dot = (aircraft_parameters.inertia_matrix) \ (-omega_tilde*(aircraft_parameters.inertia_matrix * omega) + aircraft_moments);


%put state back together
xdot = [p_dot; o_dot; V_dot; omega_dot];










end