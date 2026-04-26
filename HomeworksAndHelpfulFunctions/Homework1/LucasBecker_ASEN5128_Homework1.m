clc; close all; clear;

euler_angles = [-2;1-10;2026];
euler_angles = (pi/180) * euler_angles;

velocity_B = [13;0;2.2];
wind_B_E = [2.5;-1;-1];


%part A: angle of attack:
wind_angles = AirRelativeVelocityVectorToWindAngles(velocity_B);

%part B: ascending?
velocity_E = TransformFromBodyToInertial(velocity_B,euler_angles);
wind_E_E = TransformFromBodyToInertial(wind_B_E,euler_angles);
V_g = velocity_E + wind_E_E;


%part C: ground speed
ground_speed = norm(V_g);




