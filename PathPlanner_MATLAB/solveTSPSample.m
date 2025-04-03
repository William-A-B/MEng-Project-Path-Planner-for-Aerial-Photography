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
start_pos = [50, -20];
goal_pos = [50, 120];

wind_direction = 0;

[coordinate_path, path_cost] = solveTravellingSalesmanProblem(start_pos, goal_pos, square_corners, wind_direction);
disp(coordinate_path);
disp(path_cost);


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