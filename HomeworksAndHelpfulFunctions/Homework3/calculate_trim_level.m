function [aircraft_state_trim, control_surface_trim, trim_variable, fval] = calculate_trim_level(trim_definition,aircraft_parameters)
%Author: Lucas Becker
%Date: 1/26/2026
%ASEN 5128
%function that takes in trim definition, aircraft paramters, and wind to
%calculate the trim state and control surfaces for trim using only the trim
%definition, optimizes cost function to find trim variable
%ONLY FOR STRAIGHT, LEVEL-WING FLIGHT


%initial guess:
x0 = [0;0;0.2];

%setting constraints:
min = [-pi/4;-pi/4;0];
max = [pi/4;pi/4;1];

%using fmincon to solve optimization problem
[trim_variable, fval] = fmincon(@(x)cost_trim_level(trim_definition,x,aircraft_parameters), x0,[],[],[],[],min,max);

%determine aircraft state using trim state function
[aircraft_state_trim, control_surface_trim] = trim_state_level(trim_definition,trim_variable);





end