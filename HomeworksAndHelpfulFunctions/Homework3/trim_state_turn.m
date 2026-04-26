function [trim_state,trim_controls] = trim_state_turn(x_td,x_tv)
%Author: Lucas Becker
%Date: 1/26/2026
%Take in as inputs the trim definition and trim variable, and output the
%corresponding trim state and trim control vector
%ONLY FOR COORDINATED TURN FLIGHT

%extracting trim definition 
V = x_td(1);    %air speed
gamma = x_td(2);    %flight path angle
h = x_td(3);    %height
R = x_td(4);    %Radius

%extracting trim variables
alpha = x_tv(1);
beta = x_tv(5);
phi = x_tv(4);

%initializing outputs
trim_state = zeros(12,1);
trim_controls = zeros(4,1);

%calculating orientation angles
theta = gamma + alpha;

%calculating body velocity using windangles function and converting to
%inertial reference
V_b_E = WindAnglesToAirRelativeVelocityVector([V,beta,alpha]);

%defining X_star variable:
X_star = (V/R) * cos(gamma);


%filling trim state
trim_state(3) = -h; %height
trim_state(4) = phi;    %roll angle (phi)
trim_state(5) = theta;  %pitch angle (theta)
trim_state(6) = pi/2;
trim_state(7:9) = V_b_E;  %velocities
trim_state(10) = -X_star * sin(theta);
trim_state(11) = X_star * sin(phi) * cos(theta);
trim_state(12) = X_star * cos(phi) * cos(theta);


%filling trim controls:
trim_controls(1) = x_tv(2); %elevator input
trim_controls(2) = x_tv(6); %aileron input
trim_controls(3) = x_tv(7); %rudder input
trim_controls(4) = x_tv(3); %thrust input




end