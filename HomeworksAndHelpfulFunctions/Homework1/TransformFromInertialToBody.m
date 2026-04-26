function vector_body = TransformFromInertialToBody(vector_inertial, euler_angles)
%Author: Lucas Becker
%Date created: 1/12/2026
%For a vector in inertial coordinates, return the components in body
%coordinates
%take in inertial vector and euler angles in radians

%use Rotatin Matrix function to determine inertial to body rotation
R_EtoB = RotationMatrix321(euler_angles);

%Compute body coordinates for given inertial vector
vector_body = R_EtoB * vector_inertial;



end