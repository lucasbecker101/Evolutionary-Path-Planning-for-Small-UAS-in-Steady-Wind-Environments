function cost = cost_trim_level(trim_definition, trim_variables, aircraft_parameters)
%Author: Lucas Becker
%Date: 1/26/2026
%ASEN 5128 
%A function that takes as input the trim variable, trim definition, and
%aircraft parameters, and returns the cost (J).
%ONLY FOR STRAIGHT, LEVEL-WING FLIGHT

%ASSUMPTION FOR TRIM:
wind = [0;0;0];

%initialising xdot desired and xdot trim
x_dot_desired = zeros(12,1);

%calculating trim state and trim controls
[x_trim,controls_trim] = trim_state_level(trim_definition,trim_variables);

%using aircraft EOM function to determine xdot at trim condition
x_dot_trim = AircraftEOM([],x_trim,controls_trim,wind,aircraft_parameters);

%calculating error and cost
error = x_dot_desired - x_dot_trim;
cost = norm(error(4:12));
end