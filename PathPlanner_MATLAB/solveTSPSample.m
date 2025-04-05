%% Setup environment and algorithm constraints
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
start_pos = [50, -10];
goal_pos = [50, 120];

wind_direction = 0;

%% Solve TSP
[coordinate_path, path_cost] = solveTravellingSalesmanProblem(start_pos, goal_pos, square_corners, wind_direction);


%% Display results
figure;
hold on;
grid on;

% for i = 1:size(path_cost, 1)
%     show(path_cost(i));
% end

% Plot wind direction
% Compute wind vector components
wind_x = sin(wind_direction); 
wind_y = cos(wind_direction);

% Define arrow starting position (e.g., near the start position)
arrow_start = start_pos - [wind_x, wind_y] * 2; % Shift back slightly for clarity

% Plot the wind direction as an arrow
quiver(arrow_start(1), arrow_start(2), wind_x, wind_y, 20, 'k', 'LineWidth', 2, 'MaxHeadSize', 20, 'DisplayName', 'Wind Direction');


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