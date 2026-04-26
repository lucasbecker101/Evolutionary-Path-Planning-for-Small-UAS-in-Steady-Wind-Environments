function [trim_state,trim_controls] = trim_state_level(x_td,x_tv)
%Author: Lucas Becker
%Date: 1/26/2026
%Take in as inputs the trim definition and trim variable, and output the
%corresponding trim state and trim control vector
%ONLY FOR STRAIGHT, WINGS-LEVEL FLIGHT

%extracting trim definition 
V = x_td(1);    %air speed
gamma = x_td(2);    %flight path angle
h = x_td(3);    %height

%extracting trim variables
alpha = x_tv(1);    %angle of attack
theta = gamma + alpha;

%initializing outputs
trim_state = zeros(12,1);
trim_controls = zeros(4,1);



trim_state(3) = -h; %height
trim_state(5) = theta;  %theta



trim_state(7) = cos(alpha) * V; %u_E
trim_state(8) = 0;
trim_state(9) = sin(alpha) * V; %w_E

%control inputs
trim_controls(1) = x_tv(2);     %elevator input
trim_controls(4) = x_tv(3);     %throttle input



end