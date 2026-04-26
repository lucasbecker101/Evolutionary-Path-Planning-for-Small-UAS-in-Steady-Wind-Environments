function vector_inertial = TransformFromBodyToInertial(vector_body, euler_angles)
%Author: Lucas Becker
%Date created: 1/12/2026
%For a vector in body coordinates return the components in inertial
%coordinates

%take in inputs as a vector in the body frame and euler angles in radians

%use Rotation Matrix function to find rotation matrix, and then transpose
%to find rotation from body to inertial
R_EtoB = RotationMatrix321(euler_angles);
R_BtoE = transpose(R_EtoB);

%compute inertial vector
vector_inertial = R_BtoE * vector_body;


end