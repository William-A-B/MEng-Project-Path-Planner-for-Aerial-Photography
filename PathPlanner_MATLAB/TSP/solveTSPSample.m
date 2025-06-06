%% =========================================================================
% REAL COORDINATE TEST
polygon_vertices = [
    53.950974807525206, -1.0329672619323844, 50;
    53.946024591620926, -1.033224753997814, 50;
    53.945898302917456, -1.0243412777404899, 50;
    53.95089904334211, -1.0251566692810172, 50;
    53.950974807525206, -1.0329672619323844, 50 % Close the loop
];
% polygon_vertices = [
%     53.950974, -1.032967;
%     53.946024, -1.033224;
%     53.945898, -1.024341;
%     53.950899, -1.025156;
%     53.950974, -1.032967
% ];

polygon_vertices = round(polygon_vertices, 6);

polygon_vertices_2d = polygon_vertices(:, 1:2);

% Scale values to UTM
[utm_polygon_vertices(:, 1), utm_polygon_vertices(:, 2), utmzone] = deg2utm(polygon_vertices_2d(:, 1), polygon_vertices_2d(:, 2));

polygon_vertices_utm = [utm_polygon_vertices, polygon_vertices(:, 3)];

% Plot the polygon
figure;
plot(polygon_vertices_utm(:,2), polygon_vertices_utm(:,1), 'b-o', 'LineWidth', 2);
xlabel('Longitude');
ylabel('Latitude');
title('Defined Surveillance Area');
grid on;
axis equal;

% Define grid resolution (adjust as needed)
num_divisions_x = 7;  % Number of divisions along latitude
num_divisions_y = 7;  % Number of divisions along longitude
num_divisions_z = 3;  % Number of divisions along longitude

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

% Plot the grid points
hold on;
scatter(grid_waypoints(:,2), grid_waypoints(:,1), 'r*');
legend('Polygon Boundary', 'Grid Points');

square_size = [range(polygon_vertices_utm(:, 1))/(num_divisions_x), range(polygon_vertices_utm(:, 2))/(num_divisions_y)]; 
square_centres = grid_waypoints;
square_corners_2d = calculate_square_corner_coordinates(square_centres, square_size);
square_corners = [square_corners_2d, zeros(size(square_corners_2d, 1), 1)];

start_pos = [lat_min, lon_min, (alt_min + alt_max)/2];
goal_pos = [lat_max, lon_max, (alt_min + alt_max)/2];

%% =========================================================================
% SIMULATED AREA
% Setup environment and algorithm constraints
% Size of environment
% x_max = 100;
% y_max = 100;
% z_max = 40;
% 
% % Range of surveillance area
% x_range = [0, x_max]; % meters
% y_range = [0, y_max]; 
% z_range = [0, z_max];
% 
% square_size = [20, 20, z_max]; % Each square's dimensions
% 
% % Step size between squares
% square_step_size = square_size;
% 
% % Generate coordinates spread across the surveillance area
% [x, y, z] = ndgrid(x_range(1):square_step_size(1):x_range(2), y_range(1):square_step_size(2):y_range(2), z_range(1):square_step_size(3)/2:z_range(2));
% 
% % Coordintaes for the centres of each square
% square_centres = [x(:), y(:), z(:)];
% 
% % Coordinates for corners for each square
% square_corners_2d = calculate_square_corner_coordinates(square_centres, square_size);
% square_corners = [square_corners_2d, zeros(size(square_corners_2d, 1), 1)];
% 
% % Start and end position of UAV
% start_pos = [-10, -10];
% goal_pos = [110, 110];

%% Wind conditions
wind_direction = -pi/2;


%% Solve TSP
% tspSolver = TravellingSalesmanSolver(start_pos, goal_pos, square_corners, wind_direction);
% 
% [coordinate_path, path_cost] = tspSolver.solveTravellingSalesmanProblem(tspSolver);

[coordinate_path, dubins_path_collection] = solveTravellingSalesmanProblem(start_pos, goal_pos, square_corners, wind_direction);
display_results(start_pos, goal_pos, wind_direction, square_size(:, 1:2), square_centres, square_corners, coordinate_path, dubins_path_collection);

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