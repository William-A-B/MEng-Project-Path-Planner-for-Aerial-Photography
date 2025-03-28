% Setup environment and algorithm constraints
% Size of environment
x_max = 100;
y_max = 100;

x_range = [0, x_max]; % meters
y_range = [0, y_max]; 
square_size = [20, 20]; % Each cuboid's dimensions

margin_size = 5; % Gap between squares

% square_step_size = square_size + margin_size;
square_step_size = square_size;

[x, y] = ndgrid(x_range(1):square_step_size(1):x_range(2), y_range(1):square_step_size(2):y_range(2));

square_centres = [x(:), y(:)];

square_corners = calculate_square_corner_coordinates(square_centres, square_size);

start_pos = [-10, -10];
goal_pos = [110, 110];

% display2DGrid(start_pos, goal_pos, square_centres, square_step_size, square_corners, false)

figure;
hold on;
% Plot start and goal positions
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
plot(goal_pos(1), goal_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Goal

% Draw squares at each center position in red
num_points = size(square_centres, 1);
for i = 1:num_points
    % Define bottom-left corner of square
    bottom_left = square_centres(i,:) - square_size / 4;

    % Draw red square
    rectangle('Position', [bottom_left, square_size/2], 'EdgeColor', 'r', 'LineWidth', 1);
end

% Plot cuboid centers
scatter(square_corners(:,1), square_corners(:,2), 20, 'filled', 'b');

index = 1;
start_coord = start_pos;
num_square_centres = size(square_centres);
for i = 1:num_square_centres
    % Find the distances between the start_coord and next set of corners
    distances = sqrt(sum((square_corners(index:index+3, :) - start_coord).^2, 2));
    % Calculate the index of the closest corner
    [~, closest_corner_index] = min(distances);
    % Get the closest corner coordinate from the corner index
    closest_corner_coord = square_corners((closest_corner_index-1)+index, :);

    line([start_coord(1), closest_corner_coord(1)], [start_coord(2), closest_corner_coord(2)], 'LineWidth', 2, 'Color', 'c');

    % Update the new start coord
    start_coord = closest_corner_coord;
    index = index + 4;
end

% Labels and Grid
xlabel('X (m)');
ylabel('Y (m)');
title('Square Centers and Start/Goal Positions');
grid on;
axis equal;
hold off;


function [square_corners] = calculate_square_corner_coordinates(square_centres, square_size)
    % Calculate square corners at each center position 
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