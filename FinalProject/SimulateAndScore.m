function [score, aircraft_array, throttle_score, time_score] = SimulateAndScore(path, aircraft_parameters, control_gains, wind, xstar, ustar, obs, cost_function)
% Simulates the aircraft flying the given path using the GNC framework.
% Loops over each path segment, switching to the next segment slightly
% before the current one ends for smooth transitions.

    guidance_params.bcd = 0.5;
    guidance_params.bc  = 0.22;
    guidance_params.bhd = 0.18;
    guidance_params.bh  = 0.01;
    guidance_params.bVa = 0.13;

    Ts = 1;  % time step
    control_gains.Ts = Ts;

    waypoints = path.waypoints;
    n_wp      = size(waypoints, 1);  % includes start and target
    n_segs    = n_wp - 1;

    % --- EXTRACT TRIM CONDITIONS ---
    V_trim = xstar(7);          % trim airspeed (body x velocity)
    h_trim = -xstar(3);         % trim altitude (positive up, state is NED)

    % --- SET INITIAL STATE ---
    x_init     = xstar;
    x_init(3)  = -h_trim;      % start at trim altitude

    % Set initial yaw to match the first segment direction
    % Waypoints are 2D [North, East] — segment direction gives chi
    seg1_vec   = waypoints(2,:) - waypoints(1,:);
    chi_init   = atan2(seg1_vec(2), seg1_vec(1));  % heading angle
    x_init(6)  = chi_init;     % yaw angle (state index 6)

    % --- LINE GUIDANCE PARAMETERS ---
    line_params.X_inf  = deg2rad(45);
    line_params.k_path = 0.01;

    % How far before the waypoint to switch to the next segment (metres).
    % Starting the next segment early avoids the aircraft overshooting
    % at each corner on the straight-line path.
    lookahead_dist = V_trim * 5;  % ~5 seconds of flight ahead

    % --- INITIALISE SIMULATION ARRAYS ---
    aircraft_array = x_init;
    control_array  = ustar;
    time_iter      = 0;

    % Pre-compute grid parameters for wind lookup
    environment_size = waypoints(end, 1);
    n_wind_cells     = size(wind, 1);
    wind_cell_size   = environment_size / n_wind_cells;

    % Current segment index
    seg_idx = 1;

    % --- MAIN SIMULATION LOOP ---
    i = 1;
    max_steps = 100000;

    while seg_idx <= n_segs && i < max_steps

        % --- SAMPLE WIND AT CURRENT AIRCRAFT POSITION ---
        pos_N = aircraft_array(1, i);  % North
        pos_E = aircraft_array(2, i);  % East

        % Clamp to environment bounds
        pos_N = max(0, min(environment_size - 1e-6, pos_N));
        pos_E = max(0, min(environment_size - 1e-6, pos_E));

        % wind_mesh(row, col, :) where row = East cell, col = North cell
        wind_col = floor(pos_N / wind_cell_size) + 1;
        wind_row = floor(pos_E / wind_cell_size) + 1;
        wind_col = max(1, min(n_wind_cells, wind_col));
        wind_row = max(1, min(n_wind_cells, wind_row));

        % Extract local wind vector at this grid cell
        wind_inertial = [wind(wind_row, wind_col, 1); ...
                         wind(wind_row, wind_col, 2); ...
                         0];

        % --- BUILD CURRENT LINE SEGMENT ---
        % (rest of loop unchanged from here)

        % --- BUILD CURRENT LINE SEGMENT ---
        % r: a point on the line (start of current segment) in NED
        % q: unit direction vector of the line in NED
        p_start = waypoints(seg_idx,   :);  % [N, E]
        p_end   = waypoints(seg_idx+1, :);  % [N, E]

        seg_vec = p_end - p_start;
        seg_len = norm(seg_vec);
        if seg_len < 1e-6
            seg_idx = seg_idx + 1;
            continue;
        end
        seg_unit = seg_vec / seg_len;

        % r and q in 3D NED (flat flight — gamma = 0)
        r = [p_start(1); p_start(2); -h_trim];
        q = [seg_unit(1); seg_unit(2); 0];

        line_speed = V_trim;

        % --- SIMULATE ONE TIME STEP ON THIS SEGMENT ---
        TSPAN = Ts * [i-1, i];

        % Wind in body frame for this time step
        wind_body = TransformFromInertialToBody(wind_inertial, ...
                        aircraft_array(4:6, i));
        air_rel_vel_body = aircraft_array(7:9, i) - wind_body;
        wind_angles      = AirRelativeVelocityVectorToWindAngles(air_rel_vel_body);

        % Guidance: straight line following
        control_objectives = StraightLineGuidance( ...
                                aircraft_array(1:3, i), ...
                                line_speed, r, q, line_params);

        % Autopilot
        [control_out, x_c_out] = SLCWithFeedForwardAutopilot( ...
                                    Ts*(i-1), ...
                                    aircraft_array(:, i), ...
                                    wind_angles, ...
                                    control_objectives, ...
                                    control_gains);
        control_array(:, i)  = control_out;
        x_command(:, i)      = x_c_out;

        % Aircraft dynamics
        [TOUT, YOUT] = ode45(@(t,y) AircraftEOM(t, y, control_array(:,i), ...
                                wind_inertial, aircraft_parameters), ...
                                TSPAN, aircraft_array(:, i), []);

        aircraft_array(:, i+1) = YOUT(end, :)';
        time_iter(i+1)         = TOUT(end);
        control_array(:, i+1)  = control_out;
        x_command(:, i+1)      = x_c_out;


        % --- CHECK SEGMENT SWITCHING ---
        % Current aircraft position in 2D [N, E]
        pos_now = aircraft_array(1:2, i+1)';
        % Distance from current position to segment end waypoint
        dist_to_end = norm(p_end - pos_now);
        % Switch to next segment when within lookahead distance of the
        % end waypoint, OR if we have passed it (dot product check)
        vec_to_end    = p_end   - pos_now;
        vec_along_seg = p_end   - p_start;
        passed_end    = dot(vec_to_end, vec_along_seg) < 0;
        
        % FIX: Only increment seg_idx if we are NOT on the final segment
        if (dist_to_end < lookahead_dist || passed_end) && (seg_idx < n_segs)
            seg_idx = seg_idx + 1;
        end


        % --- CHECK TARGET REACHED ---
        % Stop simulation if we are on the final segment AND we either get
        % within a realistic capture radius (1.5x the distance traveled in one tick) 
        % OR we mathematically pass the target plane.
        dist_to_target = norm(waypoints(end,:) - pos_now);
        
        if (seg_idx == n_segs) && (dist_to_target < (V_trim * Ts * 1.5) || passed_end)
            break;
        end

        i = i + 1;
    end

    % Store simulation results in struct for scoring later
    % (scorer can read these fields when scoring is implemented)
    sim_results.aircraft_array = aircraft_array;
    sim_results.control_array  = control_array;
    sim_results.time            = time_iter;
    sim_results.n_steps         = i;


    % ----------- COST FUNCTION -----------------
    % --- time of flight ---
    % Normalize by max_steps
    time_score = 1 - (i / max_steps);

    % --- throttle use ---
    % Normalize by max possible throttle (1.0) times number of steps.
    throttle_history  = control_array(4, 1:i);
    throttle_integral = sum(abs(throttle_history)) * Ts;
    max_throttle_cost = 1.0 * i * Ts;  % worst case: full throttle every step
    throttle_score    = 1 - (throttle_integral / max_throttle_cost);


   % --- OBSTACLE AVOIDANCE ---
    environment_size  = waypoints(end, 1);
    n_obs_cells       = size(obs, 1);
    cell_size         = environment_size / n_obs_cells;

    steps_in_obstacle = 0;

    for k = 1:i+1
        pos_x = aircraft_array(1, k);  % North / x axis in your plot
        pos_y = aircraft_array(2, k);  % East  / y axis in your plot

        % Clamp positions to environment bounds before indexing
        pos_x = max(0, min(environment_size - 1e-6, pos_x));
        pos_y = max(0, min(environment_size - 1e-6, pos_y));

        % Convert to 1-based grid indices
        % col maps to x (North), row maps to y (East)
        % obs was built with obstacle_mesh(r, c) where r=y-cell, c=x-cell
        col = floor(pos_x / cell_size) + 1;  % x -> column
        row = floor(pos_y / cell_size) + 1;  % y -> row

        % Clamp to valid grid range
        col = max(1, min(n_obs_cells, col));
        row = max(1, min(n_obs_cells, row));

        if obs(row, col) == 1
            steps_in_obstacle = steps_in_obstacle + 1;
        end
    end

    % --- diagnostic: sample a few points to verify indexing ---
    % Uncomment this block temporarily to debug:
    % fprintf('  pos_x range: [%.1f, %.1f]\n', ...
    %     min(aircraft_array(1,1:i+1)), max(aircraft_array(1,1:i+1)));
    % fprintf('  pos_y range: [%.1f, %.1f]\n', ...
    %     min(aircraft_array(2,1:i+1)), max(aircraft_array(2,1:i+1)));
    % fprintf('  cell_size: %.2f, n_obs_cells: %d\n', cell_size, n_obs_cells);
    % fprintf('  steps_in_obstacle: %d / %d\n', steps_in_obstacle, i+1);

    if steps_in_obstacle == 0
        obstacle_score = 1.0;
    else
        base_collision_penalty     = -3.0;
        per_step_collision_penalty = -0.1;
        obstacle_score = base_collision_penalty + ...
                         per_step_collision_penalty * steps_in_obstacle;
    end

    % --- WEIGHTED TOTAL SCORE ---
    % obstacle term dominates: a single collision tanks the total score
    % below any collision-free path regardless of time/throttle performance
    if cost_function
        w_time     = 0.4;
        w_throttle = 0.2;
        w_obstacle = 0.4;
    else
        w_time     = 0.6;
        w_throttle = 0.0;
        w_obstacle = 0.4;
    end

    score = w_time     * time_score     + ...
            w_throttle * throttle_score + ...
            w_obstacle * obstacle_score;

    fprintf('Total Score %.4f | Time %.4f | Obstacle %.4f (steps in obs: %d) | Throttle %.4f\n', ...
            score, time_score, obstacle_score, steps_in_obstacle, throttle_score)
    fprintf('----------------------------------------------------------\n') 
end