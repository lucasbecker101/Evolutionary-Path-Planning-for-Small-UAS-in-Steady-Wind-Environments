function cost = cost_trim_turn(x_td,x_tv,aircraft_parameters)
%Author: Lucas Becker
%Date: 1/26/2026
%ASEN 5128 
%A function that takes as input the trim variable, trim definition, and
%aircraft parameters, and returns the cost (J).
%ONLY FOR COORDINATED TURN FLIGHT

%ASSUMPTION FOR TRIM:
wind = [0;0;0];

%initialising xdot desired
x_dot_desired = zeros(12,1);
x_dot_desired(6) = x_td(1)/x_td(4);

%calculating trim state and trim controls
[x_trim,controls_trim] = trim_state_turn(x_td,x_tv);

%using aircraft EOM function to determine xdot at trim condition
x_dot_trim = AircraftEOM([],x_trim,controls_trim,wind,aircraft_parameters);

%determining forces on aircraft for Y force
[density,~,~,~,~] = stdatmo(-x_trim(3));
[aircraft_forces, ~] = AircraftForcesAndMoments(x_trim, controls_trim, wind, density, aircraft_parameters);



%calculating error and cost
error = x_dot_desired - x_dot_trim;
cost = norm(error(4:12)) + aircraft_forces(2)^2;
end