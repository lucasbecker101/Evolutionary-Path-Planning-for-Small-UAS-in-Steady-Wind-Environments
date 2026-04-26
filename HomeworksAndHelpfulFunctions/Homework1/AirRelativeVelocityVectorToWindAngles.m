function [wind_angles] = AirRelativeVelocityVectorToWindAngles(velocity_body)
%Author: Lucas Becker
%Date created: 1/12/2026
%Given an air relative velocity vector in body coordinates (v_B) this
%function returns the wind angles in a column vector

%Pull out body velocity components
u = velocity_body(1);
v = velocity_body(2);
w = velocity_body(3);

%Find airspeed
V_a = sqrt(u^2 + v^2 + w^2);
beta = asin(v/V_a);
alpha = atan(w/u);

%output wind angles in radians
wind_angles = [V_a; beta; alpha];
end