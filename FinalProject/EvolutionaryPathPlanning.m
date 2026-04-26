function [best_path, best_score, best_time_score, best_throttle_score] = EvolutionaryPathPlanning(population_size, num_waypoints, iterations, min_turn_radius, wind, obs, environment_size, aircraft_parameters, control_gains, xstar, ustar, cost_function)
% this function will take in arguments about the population size, number of
% waypoints, number of iterations allowed, the minimum turn radius for
% the aircraft, and the environment details. It will then use evolutionary path planning to determine
% the best path for that environment (assumes the start location is (0,0)
% and the target location is at the end of the environment (size,size)

% --- CONSTANTS ---
start_pos  = [0, 0];
target_pos = [environment_size, environment_size];
total_dist = norm(target_pos - start_pos);
band_width = total_dist / (num_waypoints + 1);

population(population_size) = struct('waypoints', [], 'points', []);

for i = 1:population_size

    waypoints = zeros(num_waypoints + 2, 2);
    waypoints(1,   :) = start_pos;
    waypoints(end, :) = target_pos;

    for w = 2:num_waypoints + 1
        steps_remaining = num_waypoints + 2 - w;
        outer_dist = (steps_remaining + 1) * band_width;
        inner_dist =  steps_remaining      * band_width;

        overlap    = band_width * 0.2;
        outer_dist = min(total_dist, outer_dist + overlap);
        inner_dist = max(0,          inner_dist - overlap);

        t_diag    = (w - 1) / (num_waypoints + 1);
        diag_base = start_pos + t_diag * (target_pos - start_pos);

        placed       = false;
        max_attempts = 500;
        attempts     = 0;
        while ~placed && attempts < max_attempts
            attempts = attempts + 1;

            if rand() < 0.8
                sigma     = band_width * 1;
                candidate = diag_base + sigma * randn(1, 2);
            else
                candidate = [rand() * environment_size, rand() * environment_size];
            end

            candidate      = max(0, min(environment_size, candidate));
            dist_to_target = norm(target_pos - candidate);

            if dist_to_target <= outer_dist && dist_to_target >= inner_dist
                waypoints(w, :) = candidate;
                placed = true;
            end
        end

        if ~placed
            noise           = band_width * 0.1 * (rand(1,2) - 0.5);
            waypoints(w, :) = max(0, min(environment_size, diag_base + noise));
        end
    end  % end waypoint placement loop (for w)

    % Build dense point list
    all_points = [];
    for s = 1:size(waypoints, 1) - 1
        p1      = waypoints(s,   :);
        p2      = waypoints(s+1, :);
        seg_len = norm(p2 - p1);
        n_pts   = max(5, round(seg_len / 5));
        t       = linspace(0, 1, n_pts)';
        seg_pts = p1 + t .* (p2 - p1);
        if s > 1
            seg_pts = seg_pts(2:end, :);
        end
        all_points = [all_points; seg_pts]; %#ok
    end

    population(i).waypoints = waypoints;
    population(i).points    = all_points;

end  % end population initialization loop (for i)



% --------- Plot initial population to ensure functionality ---
figure('Name','Initial Population','NumberTitle','off');
hold on; axis equal; grid on;
title('Initial Population — Path Planning');
xlabel('x (m)'); ylabel('y (m)');
xlim([0, environment_size]); ylim([0, environment_size]);

cell_size = environment_size / size(obs,1);
[obs_rows, obs_cols] = find(obs == 1);
for k = 1:length(obs_rows)
    rectangle('Position',[(obs_cols(k)-1)*cell_size,(obs_rows(k)-1)*cell_size,...
              cell_size,cell_size],'FaceColor',[0.3 0.3 0.3],'EdgeColor','none');
end

colors = lines(population_size);
for i = 1:population_size
    pts = population(i).points;
    plot(pts(:,1), pts(:,2), '-', 'Color', [colors(i,:), 0.5], ...
         'LineWidth', 1.2, 'HandleVisibility', 'off');
    wp = population(i).waypoints;
    plot(wp(2:end-1,1), wp(2:end-1,2), 'o', 'Color', colors(i,:), ...
         'MarkerSize', 5, 'MarkerFaceColor', colors(i,:), ...
         'HandleVisibility', 'off');
end

plot(start_pos(1), start_pos(2),  'go', 'MarkerSize',12,'LineWidth',2.5,'DisplayName','Start');
plot(target_pos(1),target_pos(2), 'r*', 'MarkerSize',12,'LineWidth',2.5,'DisplayName','Target');
legend('Location','northwest');
hold off;

% --------- Score Initial Population -----------------
parent_scores = zeros(1, population_size);
for j = 1:population_size
    [parent_scores(j), ~] = SimulateAndScore( ...
        population(j), aircraft_parameters, control_gains, ...
        wind, xstar, ustar, obs, cost_function);
end


% ------------------ MAIN LOOP  ---------------------
for i = 1:iterations

    % Pre-allocate the expanded population slots
    population(2 * population_size) = struct('waypoints', [], 'points', []);



    % ---------------- Mutate/breed population to double it --------
    % Convert parent scores to selection probabilities
    shifted_scores = parent_scores - min(parent_scores) + 1e-6;
    selection_prob = shifted_scores / sum(shifted_scores);

    for k = 1:population_size
        if rand() > 0.4
            population(population_size + k) = mutate(population(k));
        else
            % Weighted sampling for partner based on score
            partner_idx = datasample(1:population_size, 1, 'Weights', selection_prob);
            
            % Prevent self-reproduction (optional but highly recommended)
            while partner_idx == k && population_size > 1
                partner_idx = datasample(1:population_size, 1, 'Weights', selection_prob);
            end
            
            population(population_size + k) = reproduce(population(k), population(partner_idx));
        end
    end




    % ------------------- Score population -------------------------
    score = zeros(1, length(population));
    sim_aircraft = cell(1, length(population));  % store trajectories for plotting
    time_score = zeros(1, length(population));
    throttle_score = zeros(1, length(population));

    for j = 1:length(population)
        fprintf('Iteration %i, Scoring Path %i \n', i, j)
        [score(j), sim_aircraft{j}, throttle_score(j), time_score(j)] = SimulateAndScore( ...
            population(j), aircraft_parameters, control_gains, ...
            wind, xstar, ustar, obs, cost_function);
    end
    


    % Plot expanded population after first iteration to ensure
    % functionality
    if i == 1
    % Plot full expanded population (all 40 paths) before reduction
    figure('Name', sprintf('Full Population (pre-selection) at Iteration %d', i), 'NumberTitle', 'off');
    hold on; axis equal; grid on;
    title(sprintf('Full Population — Before Selection (Iteration %d)', i));
    xlabel('x (m)'); ylabel('y (m)');
    xlim([0, environment_size]); ylim([0, environment_size]);

    % Draw obstacles
    cell_size = environment_size / size(obs, 1);
    [obs_rows, obs_cols] = find(obs == 1);
    for k = 1:length(obs_rows)
        rectangle('Position', [(obs_cols(k)-1)*cell_size, (obs_rows(k)-1)*cell_size, ...
                   cell_size, cell_size], ...
                  'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
    end

    % Draw original population in blue
    for p = 1:population_size
        pts = population(p).points;
        if isempty(pts); continue; end
        plot(pts(:,1), pts(:,2), '-', 'Color', [0.2 0.4 0.8 0.4], ...
             'LineWidth', 1.0, 'HandleVisibility', 'off');
    end

    % Draw mutated/reproduced children in orange
    for p = population_size+1 : 2*population_size
        pts = population(p).points;
        if isempty(pts); continue; end
        plot(pts(:,1), pts(:,2), '-', 'Color', [0.9 0.5 0.1 0.4], ...
             'LineWidth', 1.0, 'HandleVisibility', 'off');
    end

    % Dummy plots just to get the legend entries
    plot(nan, nan, '-', 'Color', [0.2 0.4 0.8], 'LineWidth', 2, ...
         'DisplayName', sprintf('Original (%d)', population_size));
    plot(nan, nan, '-', 'Color', [0.9 0.5 0.1], 'LineWidth', 2, ...
         'DisplayName', sprintf('Children (%d)', population_size));

    plot(start_pos(1),  start_pos(2),  'go', 'MarkerSize', 12, ...
         'LineWidth', 2.5, 'DisplayName', 'Start');
    plot(target_pos(1), target_pos(2), 'r*', 'MarkerSize', 12, ...
         'LineWidth', 2.5, 'DisplayName', 'Target');
    legend('Location', 'northwest');
    hold off;
    drawnow;
    end


    % --- REDUCE POPULATION BACK TO INITIAL SIZE ---
    % Best path (highest score) always survives. 
    % Worst path (lowest score) always removed.
    % Middle paths survive with probability proportional to their score.

    [sorted_scores, sorted_idx] = sort(score, 'descend');  % highest first

    % Best (highest score) always kept, worst (lowest score) always removed
    best_idx  = sorted_idx(1);    % first after descend sort = highest score
    worst_idx = sorted_idx(end);  % last after descend sort  = lowest score

    % Middle candidates — everyone except best and worst
    middle_idx    = sorted_idx(2:end-1);
    middle_scores = sorted_scores(2:end-1);

    % Convert scores to survival probabilities.
    % Higher score = higher probability.
    % Subtract min so the range starts at zero, then normalise.
    shifted = middle_scores - min(middle_scores) + 1e-6;
    survival_prob = shifted / sum(shifted);

    % We need (population_size - 1) survivors from the middle
    % (the -1 accounts for the best path which is guaranteed in)
    n_needed = population_size - 1;

    % Weighted sampling without replacement using the survival probabilities
    middle_survivors = datasample(middle_idx, n_needed, ...
                                  'Weights', survival_prob, ...
                                  'Replace', false);

    % Assemble final population: best + selected middle paths
    keep_idx     = [best_idx, middle_survivors];
    population   = population(keep_idx);
    sim_aircraft = sim_aircraft(keep_idx); % <--- ADD THIS LINE to keep simulations aligned
    
    % Save the scores of the surviving parents for the next generation's breeding
    parent_scores = score(keep_idx);
    parent_time_scores     = time_score(keep_idx);     % <--- ADD THIS
    parent_throttle_scores = throttle_score(keep_idx); % <--- ADD THIS


   % Plot best path and actual aircraft trajectory every 10 iterations
    if mod(i, 20) == 0
        % Find best path in current scored population
        [~, best_scored_idx] = max(parent_scores); % <--- CHANGE 'score' to 'parent_scores'
        % Find best path in current scored population
        % Score array still covers full 2*population_size at this point

        figure('Name', sprintf('Best Path at Iteration %d', i), ...
               'NumberTitle', 'off');
        hold on; axis equal; grid on;
        title(sprintf('Best Path vs Actual Trajectory — Iteration %d', i));
        xlabel('x (m)'); ylabel('y (m)');
        xlim([0, environment_size]); ylim([0, environment_size]);

        % Draw obstacles
        cell_size = environment_size / size(obs, 1);
        [obs_rows, obs_cols] = find(obs == 1);
        for k = 1:length(obs_rows)
            rectangle('Position', [(obs_cols(k)-1)*cell_size, ...
                                    (obs_rows(k)-1)*cell_size, ...
                                    cell_size, cell_size], ...
                      'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
        end

        % Draw all paths in current population (faded)
        colors = lines(length(population));
        for p = 1:length(population)
            pts = population(p).points;
            if isempty(pts); continue; end
            plot(pts(:,1), pts(:,2), '-', 'Color', [colors(p,:), 0.2], ...
                 'LineWidth', 0.8, 'HandleVisibility', 'off');
        end

        % Draw the planned waypoint path for the best scoring path
        best_wp  = population(best_scored_idx).waypoints;
        best_pts = population(best_scored_idx).points;
        plot(best_pts(:,1), best_pts(:,2), 'b-', ...
             'LineWidth', 2.0, 'DisplayName', 'Planned path');
        plot(best_wp(:,1), best_wp(:,2), 'bs', ...
             'MarkerSize', 7, 'MarkerFaceColor', 'b', ...
             'HandleVisibility', 'off');

        % Draw the actual aircraft trajectory from simulation
        % aircraft_array rows 1 and 2 are North and East positions (NED)
        ac = sim_aircraft{best_scored_idx};
        if ~isempty(ac)
            actual_N = ac(1, :);  % North
            actual_E = ac(2, :);  % East
            plot(actual_N, actual_E, 'r-', ...
                 'LineWidth', 2.0, 'DisplayName', 'Actual trajectory');

            % Mark where the aircraft ended up
            plot(actual_N(end), actual_E(end), 'r^', ...
                 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
                 'DisplayName', 'Aircraft end position');
        end

        % Start and target markers
        plot(start_pos(1),  start_pos(2),  'go', 'MarkerSize', 12, ...
             'LineWidth', 2.5, 'DisplayName', 'Start');
        plot(target_pos(1), target_pos(2), 'r*', 'MarkerSize', 12, ...
             'LineWidth', 2.5, 'DisplayName', 'Target');

        % Annotate with score
        text(environment_size * 0.02, environment_size * 0.97, ...
             sprintf('Best score: %.4f', score(best_scored_idx)), ...
             'FontSize', 10, 'Color', 'k', 'VerticalAlignment', 'top');

        legend('Location', 'southeast');
        hold off;
        drawnow;
    end
end




% % =========================================================================
% % PLOT 1: MUTATION EXAMPLE
% % =========================================================================
% % Grab a parent and create a mutated child
% parent_m      = population(1);
% child_mutated = mutate(parent_m);
% 
% figure('Name', 'Mutation Example', 'NumberTitle', 'off');
% hold on; axis equal; grid on;
% title('Path Mutation Example');
% xlabel('x (m)'); ylabel('y (m)');
% xlim([0, environment_size]); ylim([0, environment_size]);
% 
% % Draw obstacles
% cell_size = environment_size / size(obs, 1);
% [obs_rows, obs_cols] = find(obs == 1);
% for k = 1:length(obs_rows)
%     rectangle('Position', [(obs_cols(k)-1)*cell_size, (obs_rows(k)-1)*cell_size, ...
%                cell_size, cell_size], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
% end
% 
% % Plot Original Parent Path (Solid Blue)
% plot(parent_m.points(:,1), parent_m.points(:,2), '-', 'Color', [0.2 0.4 0.8], ...
%      'LineWidth', 2.0, 'DisplayName', 'Original Parent');
% plot(parent_m.waypoints(:,1), parent_m.waypoints(:,2), 'o', 'Color', [0.2 0.4 0.8], ...
%      'MarkerFaceColor', [0.2 0.4 0.8], 'MarkerSize', 6, 'HandleVisibility', 'off');
% 
% % Plot Mutated Child Path (Dashed Orange)
% if ~isempty(child_mutated.points)
%     plot(child_mutated.points(:,1), child_mutated.points(:,2), '--', 'Color', [0.9 0.5 0.1], ...
%          'LineWidth', 2.0, 'DisplayName', 'Mutated Child');
%     plot(child_mutated.waypoints(:,1), child_mutated.waypoints(:,2), 's', 'Color', [0.9 0.5 0.1], ...
%          'MarkerFaceColor', [0.9 0.5 0.1], 'MarkerSize', 6, 'HandleVisibility', 'off');
% end
% 
% % Start and target markers
% plot(0, 0, 'go', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', 'Start');
% plot(environment_size, environment_size, 'r*', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', 'Target');
% legend('Location', 'best');
% hold off;
% 
% 
% 
% % =========================================================================
% % PLOT 2: REPRODUCTION EXAMPLE
% % =========================================================================
% % Grab two parents and create a reproduced child
% parent1          = population(1);
% parent2          = population(2);
% child_reproduced = reproduce(parent1, parent2);
% 
% figure('Name', 'Reproduction Example', 'NumberTitle', 'off');
% hold on; axis equal; grid on;
% title('Path Reproduction (Crossover) Example');
% xlabel('x (m)'); ylabel('y (m)');
% xlim([0, environment_size]); ylim([0, environment_size]);
% 
% % Draw obstacles
% for k = 1:length(obs_rows)
%     rectangle('Position', [(obs_cols(k)-1)*cell_size, (obs_rows(k)-1)*cell_size, ...
%                cell_size, cell_size], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
% end
% 
% % Plot Parent 1 (Solid Blue)
% plot(parent1.points(:,1), parent1.points(:,2), '-', 'Color', [0.2 0.4 0.8 0.6], ...
%      'LineWidth', 1.5, 'DisplayName', 'Parent 1');
% plot(parent1.waypoints(:,1), parent1.waypoints(:,2), 'o', 'Color', [0.2 0.4 0.8 0.6], ...
%      'MarkerFaceColor', [0.2 0.4 0.8], 'MarkerSize', 5, 'HandleVisibility', 'off');
% 
% % Plot Parent 2 (Solid Purple)
% plot(parent2.points(:,1), parent2.points(:,2), '-', 'Color', [0.5 0.2 0.8 0.6], ...
%      'LineWidth', 1.5, 'DisplayName', 'Parent 2');
% plot(parent2.waypoints(:,1), parent2.waypoints(:,2), 'o', 'Color', [0.5 0.2 0.8 0.6], ...
%      'MarkerFaceColor', [0.5 0.2 0.8], 'MarkerSize', 5, 'HandleVisibility', 'off');
% 
% % Plot Reproduced Child (Dashed Orange, thicker line to stand out)
% if ~isempty(child_reproduced.points)
%     plot(child_reproduced.points(:,1), child_reproduced.points(:,2), '--', 'Color', [0.9 0.5 0.1], ...
%          'LineWidth', 2.5, 'DisplayName', 'Reproduced Child');
%     plot(child_reproduced.waypoints(:,1), child_reproduced.waypoints(:,2), 's', 'Color', [0.9 0.5 0.1], ...
%          'MarkerFaceColor', [0.9 0.5 0.1], 'MarkerSize', 7, 'HandleVisibility', 'off');
% end
% 
% % Start and target markers
% plot(0, 0, 'go', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', 'Start');
% plot(environment_size, environment_size, 'r*', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', 'Target');
% legend('Location', 'best');
% hold off;






% Return best path and associated scores
best_path           = population(1);
best_score          = parent_scores(1);
best_time_score     = parent_time_scores(1);
best_throttle_score = parent_throttle_scores(1);

end





