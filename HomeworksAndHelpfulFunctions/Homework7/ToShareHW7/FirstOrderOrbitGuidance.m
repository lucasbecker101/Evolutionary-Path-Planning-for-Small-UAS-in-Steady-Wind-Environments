function vel = FirstOrderOrbitGuidance(t, pos, orbit_speed, orbit_radius, orbit_center, orbit_flag, orbit_gains)
%
% STUDENTS complete this function to simulate the first order model with
% their orbit guidance algorithm
%
guidance_vector = OrbitGuidance(pos, orbit_speed, orbit_radius, orbit_center, orbit_flag, orbit_gains);

%extracting guidance vector
hc = guidance_vector(1);
hc_dot = guidance_vector(2);
Xc = guidance_vector(3);
Xc_dot = guidance_vector(4);
Vac = guidance_vector(5);

%determining velocity in each direction
x_dot = Vac * cos(Xc);
y_dot = Vac * sin(Xc);
z_dot = -hc_dot;

vel = [x_dot; y_dot; z_dot]; 



end

