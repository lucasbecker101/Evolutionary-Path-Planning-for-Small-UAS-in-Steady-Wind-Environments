function [velocity_body] = WindAnglesToAirRelativeVelocityVector(wind_angles)
%Author: Lucas Becker
%Date created: 1/12/2026
%Calculate the aircraft air relative velocity in body coordinates from the
%wind angles, outpust a three row column vector   

%inputs must come in as radians

V_a = wind_angles(1);
beta = wind_angles(2);
alpha = wind_angles(3);

velocity_body = V_a * [cos(alpha)*cos(beta); sin(beta); sin(alpha)*cos(beta)];

end