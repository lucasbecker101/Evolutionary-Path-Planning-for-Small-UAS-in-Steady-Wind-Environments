function xdot = KinematicGuidanceModel(t, x, wind_inertial, u_objective, params)
% Kinematic 1A from lecture
% Equations of motion that can be implemented in ode45
% x = [pn pe chi chi_dot h h_dot Va]
% wind_inertial = [wn, we, wd]  (northeastdown wind)
% u_objective = [h_c; h_dot_c; chi_c; chi_dot_c; Va_c]
% params:
%   bcd, bc, bhd, bh, bVa   (tuneable guidance constants, start with 1) 

%calculating yaw angle:
psi = x(3) - asin((1/x(7) * [wind_inertial(1), wind_inertial(2)] * [-sin(x(3));cos(x(3))]));

%position states:
pn_dot = x(7) * cos(psi) + wind_inertial(1);
pe_dot = x(7) * sin(psi) + wind_inertial(2);

%course angle states:
chi_dot = x(4);
chi_d_dot = params.bcd * (u_objective(4) - x(4)) + params.bc * (u_objective(3) - x(3));

%height states:
h_dot = x(6);
h_d_dot = params.bhd * (u_objective(2) - x(6)) + params.bh * (u_objective(1) - x(5));

%airspeed:
Va_dot = params.bVa * (u_objective(5) - x(7));

xdot = [pn_dot; pe_dot; chi_dot; chi_d_dot; h_dot; h_d_dot; Va_dot];

end