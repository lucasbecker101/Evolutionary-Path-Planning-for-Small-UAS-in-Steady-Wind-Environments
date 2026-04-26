function new_path = mutate(path)
% Takes a path, selects a random interior waypoint, moves it to a new
% location following all placement rules, reconnects with straight lines.

    new_path = path;
    waypoints = path.waypoints;
    n_wp = size(waypoints, 1);  % includes start and target

    % Only interior waypoints can be mutated (not index 1 or n_wp)
    num_interior = n_wp - 2;
    if num_interior < 1
        return;
    end

    % Pick a random interior waypoint to move
    % Index in waypoints array is w (2 to n_wp-1)
    w = randi([2, n_wp-1]);

    % --- FIND VALID NEW POSITION FOR THIS WAYPOINT ---
    % Rules (same as initialization):
    %   1. Must be inside environment bounds
    %   2. Must be closer to target than the previous waypoint
    %   3. Must be farther from target than the next waypoint
    %      (so the monotonic-progress chain is preserved on both sides)

    % Infer environment size from the target position
    % (target is always at [environment_size, environment_size])
    environment_size = waypoints(end, 1);
    target_pos = waypoints(end, :);

    prev_dist = norm(target_pos - waypoints(w-1, :));  % must be less than this
    next_dist = norm(target_pos - waypoints(w+1, :));  % must be greater than this

    % Infer a scatter range from the spacing between neighbors
    % Use the average distance between surrounding waypoints as the
    % perturbation scale so mutations are locally meaningful
    neighbor_span = norm(waypoints(w+1,:) - waypoints(w-1,:));
    scatter_range = neighbor_span * 0.6;

    % Try to place the waypoint near its current position but perturbed
    placed = false;
    new_wp = waypoints(w, :);  % fallback: no change

    for attempt = 1:500
        % Perturb around the current waypoint position
        candidate = waypoints(w,:) + scatter_range * (2*rand(1,2) - 1);

        % Check all rules
        in_bounds   = all(candidate >= 0) && all(candidate <= environment_size);
        dist_to_tgt = norm(target_pos - candidate);
        closer_than_prev = dist_to_tgt < prev_dist;
        farther_than_next = dist_to_tgt > next_dist;

        if in_bounds && closer_than_prev && farther_than_next
            new_wp = candidate;
            placed = true;
            break;
        end
    end

    if ~placed
        % Fallback: place on the straight line between neighbors with noise
        t         = 0.3 + rand() * 0.4;  % somewhere in the middle 40%
        base      = waypoints(w-1,:) + t * (waypoints(w+1,:) - waypoints(w-1,:));
        new_wp    = max(0, min(environment_size, base));
    end

    % Update the waypoint
    new_path.waypoints(w, :) = new_wp;

    % --- RECONNECT ALL SEGMENTS WITH STRAIGHT LINES ---
    % Rebuild the full dense point list from scratch using the new waypoints
    all_points = [];
    new_waypoints = new_path.waypoints;

    for s = 1:size(new_waypoints, 1) - 1
        p1      = new_waypoints(s,   :);
        p2      = new_waypoints(s+1, :);
        seg_len = norm(p2 - p1);
        n_pts   = max(5, round(seg_len / 5));
        t_line  = linspace(0, 1, n_pts)';
        seg_pts = p1 + t_line .* (p2 - p1);
        if s > 1
            seg_pts = seg_pts(2:end, :);
        end
        all_points = [all_points; seg_pts]; %#ok
    end

    new_path.points = all_points;
end