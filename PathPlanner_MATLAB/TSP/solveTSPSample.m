%% =========================================================================
% REAL COORDINATE TEST
% polygon_vertices = [
%     53.950974807525206, -1.0329672619323844;
%     53.946024591620926, -1.033224753997814;
%     53.945898302917456, -1.0243412777404899;
%     53.95089904334211, -1.0251566692810172;
%     53.950974807525206, -1.0329672619323844 % Close the loop
% ];
% polygon_vertices = [
%     53.950974, -1.032967;
%     53.946024, -1.033224;
%     53.945898, -1.024341;
%     53.950899, -1.025156;
%     53.950974, -1.032967
% ];
% 
% % Scale values to UTM
% [polygon_vertices(:, 1), polygon_vertices(:, 2), utmzone] = deg2utm(polygon_vertices(:, 1), polygon_vertices(:, 2));
% 
% 
% % Plot the polygon
% figure;
% plot(polygon_vertices(:,2), polygon_vertices(:,1), 'b-o', 'LineWidth', 2);
% xlabel('Longitude');
% ylabel('Latitude');
% title('Defined Surveillance Area');
% grid on;
% axis equal;
% 
% % Define grid resolution (adjust as needed)
% num_divisions_x = 7;  % Number of divisions along latitude
% num_divisions_y = 7;  % Number of divisions along longitude
% 
% lat_min = min(polygon_vertices(:,1));
% lat_max = max(polygon_vertices(:,1));
% lon_min = min(polygon_vertices(:,2));
% lon_max = max(polygon_vertices(:,2));
% 
% % Generate a grid of waypoints
% lat_values = linspace(lat_min, lat_max, num_divisions_x);
% lon_values = linspace(lon_min, lon_max, num_divisions_y);
% [lon_grid, lat_grid] = ndgrid(lon_values, lat_values);
% 
% % Convert to list of waypoints
% grid_waypoints = [lat_grid(:), lon_grid(:)];
% 
% % Plot the grid points
% hold on;
% scatter(grid_waypoints(:,2), grid_waypoints(:,1), 'r*');
% legend('Polygon Boundary', 'Grid Points');
% 
% square_size = [range(polygon_vertices(:, 1))/(num_divisions_x*1), range(polygon_vertices(:, 2))/(num_divisions_y*1)]; 
% square_centres = grid_waypoints;
% square_corners = calculate_square_corner_coordinates(square_centres, square_size);
% 
% start_pos = [lat_min, lon_min];
% goal_pos = [lat_max, lon_max];

%% =========================================================================
% SIMULATED AREA
% Setup environment and algorithm constraints
% Size of environment
x_max = 100;
y_max = 100;

% Range of surveillance area
x_range = [0, x_max]; % meters
y_range = [0, y_max]; 

square_size = [20, 20]; % Each square's dimensions

% Step size between squares
square_step_size = square_size;

% Generate coordinates spread across the surveillance area
[x, y] = ndgrid(x_range(1):square_step_size(1):x_range(2), y_range(1):square_step_size(2):y_range(2));

% Coordintaes for the centres of each square
square_centres = [x(:), y(:)];

% Coordinates for corners for each square
square_corners = calculate_square_corner_coordinates(square_centres, square_size);

% Start and end position of UAV
start_pos = [-10, -10];
goal_pos = [110, 110];

%% Wind conditions
wind_direction = -pi/4;


%% Solve TSP
% tspSolver = TravellingSalesmanSolver(start_pos, goal_pos, square_corners, wind_direction);
% 
% [coordinate_path, path_cost] = tspSolver.solveTravellingSalesmanProblem(tspSolver);

[coordinate_path, dubins_path_collection] = solveTravellingSalesmanProblem(start_pos, goal_pos, square_corners, wind_direction);


%% Display results
figure;
hold on;
grid on;

for i = 1:size(dubins_path_collection, 1)
    % show(paths{i});
    disp(i)

    % Evaluate pathSegObj to get path states (poses)
    interpStates = interpolate(dubins_path_collection{i}, linspace(0, dubins_path_collection{i}.Length, 50));
    
    % Plot only the path line (fast)
    plot(interpStates(:,1), interpStates(:,2), 'b-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
end

% Plot wind direction
% Compute wind vector components
wind_x = sin(wind_direction); 
wind_y = cos(wind_direction);

% Define arrow starting position (e.g., near the start position)
arrow_start = start_pos - [wind_x, wind_y] * 2; % Shift back slightly for clarity

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
    bottom_left = square_centres(i,:) - square_size / 4;
    % Draw red square
    rectangle('Position', [bottom_left, square_size/2], 'EdgeColor', 'c', 'LineWidth', 1);
end

% Plot cuboid centres
scatter(square_corners(:,1), square_corners(:,2), 20, 'filled', 'b', 'DisplayName', 'Imaging Positions');


plot(coordinate_path(:,1), coordinate_path(:,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');

% Add legend to clarify the wind direction
legend('show');

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

function [square_corner_coords] = new_calculate_square_corner_coordinates(square_centres, square_size)
    % Calculate square corners from each central square position 
    num_squares = size(square_centres, 1);
    square_radius = square_size(1) / 4;
    square_corner_coords = [];
    for i = 1:num_squares
        centre = square_centres(i, :);
        corners = [centre(1) - square_radius, centre(2) + square_radius;
                   centre(1) + square_radius, centre(2) + square_radius;
                   centre(1) + square_radius, centre(2) - square_radius;
                   centre(1) - square_radius, centre(2) - square_radius];
        square_corner_coords{i} = corners;
    end
end