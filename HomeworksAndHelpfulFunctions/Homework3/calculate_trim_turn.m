function [aircraft_state_trim, control_surface_trim, fval] = calculate_trim_turn(x_td,aircraft_parameters)
%Author: Lucas Becker
%Date: 1/26/2026
%ASEN 5128
%function that takes in trim definition, aircraft paramters, and wind to
%calculate the trim state and control surfaces for trim using only the trim
%definition, optimizes cost function to find trim variable
%ONLY FOR COORDINATED TURN FLIGHT

%initial guess:
phi0 = atan(x_td(1)^2/(9.81*x_td(4)));

x0 = [5*pi/180; 0; 0.3; phi0; 0; 0; 0];


%setting constraints:
min = [-20; -25; 0; -70; -10; -20; -20]*pi/180;
max = [ 20;  25; 1;  70;  10;  20;  20]*pi/180;
min(3) = 0;
max(3) = 1;

%using fmincon to solve optimization problem
[trim_variable, fval] = fmincon(@(x)cost_trim_turn(x_td,x,aircraft_parameters), x0,[],[],[],[],min,max);

%determine aircraft state using trim state function
[aircraft_state_trim, control_surface_trim] = trim_state_turn(x_td,trim_variable);


end