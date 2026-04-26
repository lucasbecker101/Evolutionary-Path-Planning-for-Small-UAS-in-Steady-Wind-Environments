function control_objectives = StraightLineGuidance(x,line_speed,r, q, params)
% x = position
% xdot = velocity
% control_objectives = [h_c; h_dot_c; chi_c; chi_dot_c; Va_c]

X_q = atan2(q(2),q(1));

%distance from starting point (assumes position coordinates in NED inertial
%frame)
dpn = x(1) - r(1);
dpe = x(2) - r(2);

%calculating cross track error and along track distance
e_px = cos(X_q) * dpn + sin(X_q) * dpe;
e_py = -sin(X_q) * dpn + cos(X_q) * dpe;

%setting commanded course angle and course angle rate
chi_c = X_q - params.X_inf * (2 / pi) * atan(params.k_path * e_py);
chi_dot_c = 0;


% setting commanded altitude based on horizontal progress along the line:
q_horiz = sqrt(q(1)^2 + q(2)^2);
h_c = -r(3) - e_px * q(3)/q_horiz;

%commanding feed forward climb rate:
h_dot_c = -line_speed * q(3)/q_horiz;

%setting commanded airspeed
Va_c = line_speed;

control_objectives = [h_c; h_dot_c; chi_c; chi_dot_c; Va_c];
end