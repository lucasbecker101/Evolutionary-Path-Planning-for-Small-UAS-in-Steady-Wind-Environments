function new_path = reproduce(path1, path2)
% Takes two paths and mixes their interior waypoints to create a new path.
% The new path inherits waypoints from either parent at each position,
% then fixes any monotonicity violations before reconnecting.

    new_path = path1;
    wp1 = path1.waypoints;
    wp2 = path2.waypoints;

    n_wp = size(wp1, 1);  % includes start and target

    % If paths have different numbers of waypoints, just return path1
    if size(wp2, 1) ~= n_wp
        return;
    end

    num_interior = n_wp - 2;
    if num_interior < 1
        return;
    end

    environment_size = wp1(end, 1);
    target_pos       = wp1(end, :);

    % --- BUILD MIXED INTERIOR WAYPOINTS ---
    % For each interior waypoint position, randomly pick from either parent.
    % This is standard single-point or uniform crossover — we use uniform
    % (independent coin flip per waypoint) for maximum diversity.
    new_waypoints        = wp1;
    new_waypoints(1,  :) = wp1(1,   :);  % start fixed
    new_waypoints(end,:) = wp1(end,  :);  % target fixed

    for w = 2:n_wp - 1
        if rand() > 0.5
            new_waypoints(w,:) = wp2(w,:);
        else
            new_waypoints(w,:) = wp1(w,:);
        end
    end

    % --- ENFORCE MONOTONIC PROGRESS ---
    % After mixing, the chain may have violations where a waypoint is
    % farther from the target than the previous one.
    % Walk forward through the waypoints and fix any violation by
    % replacing the offending waypoint with a valid random candidate
    % between its neighbors.
    for w = 2:n_wp - 1
        prev_dist = norm(target_pos - new_waypoints(w-1, :));
        next_dist = norm(target_pos - new_waypoints(w+1, :));
        cur_dist  = norm(target_pos - new_waypoints(w,   :));

        % Valid if strictly between prev and next in distance to target
        is_valid = cur_dist < prev_dist && cur_dist > next_dist;

        if ~is_valid
            % Try to find a valid replacement near the line between neighbors
            placed = false;
            neighbor_span = norm(new_waypoints(w+1,:) - new_waypoints(w-1,:));
            scatter_range = neighbor_span * 0.5;

            for attempt = 1:500
                candidate   = new_waypoints(w,:) + scatter_range * (2*rand(1,2) - 1);
                in_bounds   = all(candidate >= 0) && all(candidate <= environment_size);
                dist_to_tgt = norm(target_pos - candidate);

                if in_bounds && dist_to_tgt < prev_dist && dist_to_tgt > next_dist
                    new_waypoints(w,:) = candidate;
                    placed = true;
                    break;
                end
            end

            if ~placed
                % Fallback: interpolate between neighbors at a random t
                t = 0.3 + rand() * 0.4;
                base = new_waypoints(w-1,:) + t * (new_waypoints(w+1,:) - new_waypoints(w-1,:));
                new_waypoints(w,:) = max(0, min(environment_size, base));
            end
        end
    end

    % --- RECONNECT WITH STRAIGHT LINES ---
    all_points = [];
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

    new_path.waypoints = new_waypoints;
    new_path.points    = all_points;
end