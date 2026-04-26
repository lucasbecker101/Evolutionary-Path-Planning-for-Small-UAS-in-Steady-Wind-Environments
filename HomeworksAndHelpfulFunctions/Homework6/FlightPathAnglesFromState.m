function [flight_angles] = FlightPathAnglesFromState(x)
%Author: Lucas Becker
%Date: 2/17/2026
% Extract position and v_inert from state vector
velocity = x(7:9);
orientation = x(4:6);

%convert v_inert from body frame to inertial frame:
v_inert = TransformFromBodyToInertial(velocity,orientation);

% Calculate flight path angles
flight_angles(1) = norm(v_inert);
flight_angles(2) = atan2(v_inert(2), v_inert(1)); % Chi
flight_angles(3) = atan2(v_inert(3), sqrt(v_inert(1)^2 + v_inert(2)^2)); % Gamma

flight_angles = flight_angles';

end