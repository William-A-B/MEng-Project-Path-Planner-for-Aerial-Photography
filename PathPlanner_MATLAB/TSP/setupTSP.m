function [coordinate_path, dubins_path_waypoints] = setupTSP(start_pos, goal_pos, polygon_vertices, wind_direction, altitude_limits, num_divisions, plot_results)

    %% =========================================================================
    % REAL COORDINATE TEST
    polygon_vertices = round(polygon_vertices, 6);
    start_pos = round(start_pos, 6);
    goal_pos = round(goal_pos, 6);
    
    polygon_vertices_2d = polygon_vertices(:, 1:2);
    
    % Scale values to UTM
    [utm_polygon_vertices(:, 1), utm_polygon_vertices(:, 2), polygon_utmzone] = deg2utm(polygon_vertices_2d(:, 1), polygon_vertices_2d(:, 2));
    
    [start_pos_utm(:, 1), start_pos_utm(:, 2), start_pos_utmzone] = deg2utm(start_pos(:, 1), start_pos(:, 2));
    start_pos_utm = [start_pos_utm, start_pos(:, 3)];
    [goal_pos_utm(:, 1), goal_pos_utm(:, 2), goal_pos_utmzone] = deg2utm(goal_pos(:, 1), goal_pos(:, 2));
    goal_pos_utm = [goal_pos_utm, goal_pos(:, 3)];

    polygon_vertices_utm = [utm_polygon_vertices, polygon_vertices(:, 3)];
    
    % Define grid resolution (adjust as needed)
    num_divisions_x = num_divisions.x;  % Number of divisions along latitude
    num_divisions_y = num_divisions.y;  % Number of divisions along longitude
    num_divisions_z = num_divisions.z;  % Number of divisions along longitude
    
    lat_min = min(polygon_vertices_utm(:,1));
    lat_max = max(polygon_vertices_utm(:,1));
    lon_min = min(polygon_vertices_utm(:,2));
    lon_max = max(polygon_vertices_utm(:,2));
    alt_min = min(polygon_vertices_utm(:,3));
    alt_max = max(polygon_vertices_utm(:,3));
    
    % Generate a grid of waypoints
    lat_values = linspace(lat_min, lat_max, num_divisions_x);
    lon_values = linspace(lon_min, lon_max, num_divisions_y);
    alt_values = linspace(alt_min, alt_max, num_divisions_z);
    [lon_grid, lat_grid, alt_grid] = ndgrid(lon_values, lat_values, alt_values);
    
    % Convert to list of waypoints
    grid_waypoints = [lat_grid(:), lon_grid(:), alt_grid(:)];

    % Remove any points outside the polygon region
    [in, on] = inpolygon(grid_waypoints(:, 1), grid_waypoints(:, 2), polygon_vertices_utm(:,1), polygon_vertices_utm(:,2));
    
    valid_coordinates_indices = in | on;
    valid_grid_waypoints = grid_waypoints(valid_coordinates_indices, :);

    % start_pos = [lat_min, lon_min, (alt_min + alt_max)/2];
    % goal_pos = [lat_max, lon_max, (alt_min + alt_max)/2];

    if plot_results
        display_initial_coordinate_grid(polygon_vertices_utm, grid_waypoints, valid_grid_waypoints);
    end
    
    square_size = [range(polygon_vertices_utm(:, 1))/(num_divisions_x), range(polygon_vertices_utm(:, 2))/(num_divisions_y)]; 
    square_centres = valid_grid_waypoints;
    square_corners_2d = calculate_square_corner_coordinates(square_centres, square_size);
    square_corners = [square_corners_2d, zeros(size(square_corners_2d, 1), 1)];
    
    %% Solve TSP
    [coordinate_path, dubins_path_collection] = solveTravellingSalesmanProblem(start_pos_utm, goal_pos_utm, square_corners, wind_direction);
    if plot_results
        display_results(start_pos_utm, goal_pos_utm, wind_direction, square_size(:, 1:2), square_centres, square_corners, coordinate_path, dubins_path_collection);
    end
    
    dubins_path_waypoints_utm = [];

    for i = 1:size(dubins_path_collection, 1)
        % Evaluate pathSegObj to get path states (poses)
        dubins_path_waypoints_connection = interpolate(dubins_path_collection{i}, linspace(0, dubins_path_collection{i}.Length, 20));
        dubins_path_waypoints_utm = [dubins_path_waypoints_utm; dubins_path_waypoints_connection];
    end
    
    % Update the utmzone array to equal the size of the full list of
    % coordinates
    num_coordinate_points = size(coordinate_path, 1);
    utmzone_updated = repmat(polygon_utmzone(1, :), num_coordinate_points, 1);
    [resulting_lat_coords, resulting_lon_coords] = utm2deg(coordinate_path(:, 1), coordinate_path(:, 2), utmzone_updated);
    coordinate_path = [resulting_lat_coords, resulting_lon_coords];

    num_dubins_waypoints = size(dubins_path_waypoints_utm, 1);
    utmzone_dubins_waypoints = repmat(polygon_utmzone(1, :), num_dubins_waypoints, 1);
    [dubins_lat_coords, dubins_lon_coords] = utm2deg(dubins_path_waypoints_utm(:, 1), dubins_path_waypoints_utm(:, 2), utmzone_dubins_waypoints);
    dubins_path_waypoints = [dubins_lat_coords, dubins_lon_coords];
    
end

%% HELPER FUNCTIONS

function [square_corners] = calculate_square_corner_coordinates(square_centres, square_size)
    % Calculate square corners from each central square position 
    num_squares = size(square_centres, 1);
    square_radius = square_size(1) / 4;
    square_corners = zeros(num_squares * 4, 2); % 4 corners per square
    index = 1;
    for i = 1:num_squares
        centre = square_centres(i, :);
        corners = [centre(1) - square_radius, centre(2) + square_radius;
                   centre(1) + square_radius, centre(2) + square_radius;
                   centre(1) + square_radius, centre(2) - square_radius;
                   centre(1) - square_radius, centre(2) - square_radius];
        square_corners(index:index+3, :) = corners;
        index = index + 4;
    end
end

function display_results(start_pos, goal_pos, wind_direction, square_size, square_centres, square_corners, coordinate_path, dubins_paths)
    % Display results
    figure;
    hold on;
    grid on;
    
    for i = 1:size(dubins_paths, 1)
        % show(paths{i});
        % Evaluate pathSegObj to get path states (poses)
        interpStates = interpolate(dubins_paths{i}, linspace(0, dubins_paths{i}.Length, 50));

        % Plot only the path line (fast)
        plot(interpStates(:,1), interpStates(:,2), 'b-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    
    % Plot wind direction
    % Compute wind vector components
    wind_x = sin(wind_direction); 
    wind_y = cos(wind_direction);
    
    % Define arrow starting position (e.g., near the start position)
    arrow_start = start_pos(1, 1:2) - [wind_x, wind_y] * 2; % Shift back slightly for clarity
    
    % Plot the wind direction as an arrow
    % quiver(arrow_start(1), arrow_start(2), wind_x, wind_y, range(polygon_vertices(:, 1))/5, 'k', 'LineWidth', 2, 'MaxHeadSize', 20, 'DisplayName', 'Wind Direction');
    quiver(arrow_start(1), arrow_start(2), wind_x, wind_y, 40, 'k', 'LineWidth', 2, 'MaxHeadSize', 20, 'DisplayName', 'Wind Direction');
    
    
    % Plot start and goal positions
    plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Position'); % Start
    plot(goal_pos(1), goal_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal Position'); % Goal
    
    % Draw squares at each center position in red
    num_points = size(square_centres, 1);
    for i = 1:num_points
        % Define bottom-left corner of square
        bottom_left = square_centres(i, 1:2) - square_size / 4;
        % Draw red square
        rectangle('Position', [bottom_left, square_size/2], 'EdgeColor', 'c', 'LineWidth', 1);
    end
    
    % Plot cuboid centres
    scatter(square_corners(:,1), square_corners(:,2), 20, 'filled', 'b', 'DisplayName', 'Imaging Positions');
    
    
    plot(coordinate_path(:,1), coordinate_path(:,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');
    
    % Add legend to clarify the wind direction
    legend('show');
end

function display_initial_coordinate_grid(polygon_vertices_utm, grid_waypoints, valid_grid_waypoints)
    % Plot the polygon
    figure;
    hold on;
    % Plot polygon
    plot(polygon_vertices_utm(:,1), polygon_vertices_utm(:,2), 'b-o', 'LineWidth', 2);
    xlabel('Longitude');
    ylabel('Latitude');
    title('Defined Surveillance Area');
    grid on;
    axis equal;

    % Plot the grid points
    scatter(grid_waypoints(:,1), grid_waypoints(:,2), 'r*');
    scatter(valid_grid_waypoints(:,1), valid_grid_waypoints(:,2), 'g*');
    legend('Polygon Boundary', 'Grid Points');
end