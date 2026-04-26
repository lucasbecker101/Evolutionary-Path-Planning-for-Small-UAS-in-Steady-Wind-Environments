function [wind_mesh, obstacle_mesh] = get_environment(size, n_obstacles, max_wind, discretization, seed)
% this function will take in parameters describing an environment and
% create a psuedo-random environment that fits those parameters with given
% obstacles, wind paths, discretization and size
%
% Inputs:
%   size:            side lengths of square environment in (m)
%   n_obstacles:     number of obstacles present in the region
%   max_wind:        max wind allowed in environment (m/s)
%   discretization:  spatial resolution/step size to divide the environment (m/cell)
%   seed:            (optional) integer seed for the RNG. If provided, the
%                    environment is fully repeatable. If omitted, a random
%                    environment is generated each call.
%
% Outputs:
%   wind_mesh:     contains inertial wind vector for every cell, constant
%                  throughout each cell (m/s)
%   obstacle_mesh: contains information for each cell on whether there is
%                  an obstacle in that cell (1 = obstacle, 0 = free)

   
    % controlling randomness for repeatability in inital testing
    previous_rng = rng;
    if nargin == 5
        rng(seed, 'twister');
    end

    % Use a try/finally block so the RNG is always restored, even if an
    % error is thrown mid-function.
    try

        % Calculate the number of grid cells
        num_cells = round(size / discretization);

        % Initialize meshes
        wind_mesh     = zeros(num_cells, num_cells, 2);
        obstacle_mesh = zeros(num_cells, num_cells);

        % --- OBSTACLE GENERATION ---

        buffer       = 2  ;
        max_attempts = 1000;
        min_dim      = 2;
        max_dim      = max(3, round(num_cells * 0.15));

        for i = 1:n_obstacles
            placed   = false;
            attempts = 0;

            while ~placed && attempts < max_attempts
                attempts = attempts + 1;

                w = randi([min_dim, max_dim]);
                h = randi([min_dim, max_dim]);
                r = randi([1 + buffer, num_cells - h - buffer]);
                c = randi([1 + buffer, num_cells - w - buffer]);

                check_zone = obstacle_mesh(r - buffer : r + h + buffer, ...
                                           c - buffer : c + w + buffer);

                if all(check_zone(:) == 0)
                    obstacle_mesh(r : r + h - 1, c : c + w - 1) = 1;
                    placed = true;
                end
            end

            if attempts == max_attempts
                warning('Could not place obstacle %d due to space/buffer constraints.', i);
            end
        end

        % --- WIND GENERATION ---

        global_angle = rand() * 2 * pi;
        base_Vx      = cos(global_angle);
        base_Vy      = sin(global_angle);

        coarse_grid_size  = 5;
        perturbation_scale = 0.3;

        [X_coarse, Y_coarse] = meshgrid(linspace(1, num_cells, coarse_grid_size));
        [X_fine,   Y_fine  ] = meshgrid(1:num_cells, 1:num_cells);

        Vx_coarse_noise = randn(coarse_grid_size, coarse_grid_size) * perturbation_scale;
        Vy_coarse_noise = randn(coarse_grid_size, coarse_grid_size) * perturbation_scale;

        Vx_noise = interp2(X_coarse, Y_coarse, Vx_coarse_noise, X_fine, Y_fine, 'spline');
        Vy_noise = interp2(X_coarse, Y_coarse, Vy_coarse_noise, X_fine, Y_fine, 'spline');

        Vx_fine = base_Vx + Vx_noise;
        Vy_fine = base_Vy + Vy_noise;

        wind_mag    = sqrt(Vx_fine.^2 + Vy_fine.^2);
        current_max = max(wind_mag(:));

        if current_max > 0
            Vx_scaled = (Vx_fine ./ current_max) .* max_wind;
            Vy_scaled = (Vy_fine ./ current_max) .* max_wind;
        else
            Vx_scaled = Vx_fine;
            Vy_scaled = Vy_fine;
        end

        Vx_scaled(obstacle_mesh == 1) = 0;
        Vy_scaled(obstacle_mesh == 1) = 0;

        wind_mesh(:, :, 1) = Vx_scaled;
        wind_mesh(:, :, 2) = Vy_scaled;

    catch ME
        rng(previous_rng);   % restore before re-throwing
        rethrow(ME);
    end

    % Restore the RNG state
    rng(previous_rng);

end