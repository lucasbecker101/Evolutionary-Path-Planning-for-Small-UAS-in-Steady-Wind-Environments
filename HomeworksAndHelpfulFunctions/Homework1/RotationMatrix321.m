function R = RotationMatrix321(euler_angles)
%Author: Lucas Becker
%Date created: 1/12/2026
%Calculate the 3-2-1 rotation matrix given the Euler Angles

%euler angles should come in the form [phi(roll),theta(pitch),psi(yaw)], in
%radians 

%OUTPUTS an inertial to body rotations

%pull out euler angles
phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

%determine rotation matrices to temporary frames
R_1 = [1,0,0;
       0,cos(phi),sin(phi);
       0,-sin(phi),cos(phi)];
R_2 = [cos(theta),0,-sin(theta);
       0,1,0;
       sin(theta),0,cos(theta)];
R_3 = [cos(psi), sin(psi), 0;
       -sin(psi), cos(psi),0;
       0,0,1];

%calculate 3-2-1 rotation matrix
R = R_1 * R_2 * R_3;




end