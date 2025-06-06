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
    rectangle('Position', [bottom_left, square_size/2], 'EdgeColor', 'c', 'LineWidth', 1);
end

% Plot cuboid centres
scatter(square_corners(:,1), square_corners(:,2), 20, 'filled', 'b');

% SETUP ALGORITHM PARAMETERS
% Square corner index
index = 1;
% Define starting coordinate
start_coord = start_pos;
% Number of squares to search (== num images to take)
num_squares = size(square_centres, 1);

% Determine grid row length
square_row_length = sqrt(num_squares);

% Initialise array for storing coordinate path
coordinate_path = zeros(num_squares+2, 2);
coordinate_path(1, :) = start_pos;

waypoint_path = [];
num_waypoints_per_coord = 20;

% Start by moving left to right
left_to_right = true;

% Initialise Dubins path algorithm
connectionObj = uavDubinsConnection;
connectionObj.MaxRollAngle = 1.1;

% Direction UAV arrived at the previous square corner
prev_arrival_dir = 0;

for i = 1:(num_squares+1)
    % If all squares calculated for, find path to goal position
    if i == (num_squares + 1)
        closest_corner_coord = goal_pos;
    else
        % Find the distances between the start_coord and next set of corners
        distances = sqrt(sum((square_corners(index:index+3, :) - start_coord).^2, 2));
        % Calculate the index of the closest corner
        [~, closest_corner_index] = min(distances);
        % Get the closest corner coordinate from the corner index
        closest_corner_coord = square_corners((closest_corner_index-1)+index, :);
    end
    
    % Calculate departure and arrival directions between each square
    [departure_dir, arrival_dir] = calculate_directions(start_coord, closest_corner_coord);
    
    % Calculate start and goal position for dubins connection
    start_pose = [start_coord, 0, prev_arrival_dir];
    goal_pose = [closest_corner_coord, 0, arrival_dir];
    prev_arrival_dir = arrival_dir;
    % Calculate Dubins Path and display on plot
    [pathSegObj, pathCosts] = connect(connectionObj, start_pose, goal_pose);
    show(pathSegObj{1})

    dubinsPath = pathSegObj{1};
    segment_waypoints = interpolate(dubinsPath, linspace(0, dubinsPath.Length, num_waypoints_per_coord));

    waypoint_path = [waypoint_path; segment_waypoints];

    % Update coordinate path array
    coordinate_path(i+1, :) = closest_corner_coord;
    % Update the new start coord
    start_coord = closest_corner_coord;
    
    % Detect end of row
    if mod(i, (square_row_length)) == 0
        % Switch row direction
        left_to_right = ~left_to_right;
        % Move up/down to the next row
        next_row_index = index + square_row_length * 4; % Move to next row
        if next_row_index <= size(square_corners, 1)
            index = next_row_index;
            continue; % Skip to next iteration
        end
    end

    % Move to the next square
    if left_to_right
        index = index + 4;  % Move forward
    else
        index = index - 4;  % Move backward
    end
end


% Labels and Grid
xlabel('X (m)');
ylabel('Y (m)');
title('UAV Shortest Path Through Each Image Location');
grid on;
axis equal;
hold off;

%[image_coordinates, waypoints] = smooth_coordinate_path(coordinate_path, waypoint_path, num_waypoints_per_coord);
[smoothed_coordinates, smoothed_waypoints] = smooth_coordinate_path_v2(coordinate_path, waypoint_path, num_waypoints_per_coord);

figure; hold on;
grid on; axis equal;
title('Original vs. Smoothed UAV Path');

% Plot original path
plot(coordinate_path(:,1), coordinate_path(:,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');

% Plot smoothed path
plot(smoothed_coordinates(:,1), smoothed_coordinates(:,2), 'b-o', 'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Smoothed Path');

% Plot original waypoints
scatter(waypoint_path(:,1), waypoint_path(:,2), 10, 'r', 'filled', 'DisplayName', 'Original Waypoints');

% Plot smoothed waypoints
scatter(smoothed_waypoints(:,1), smoothed_waypoints(:,2), 10, 'b', 'filled', 'DisplayName', 'Smoothed Waypoints');

% Labels and legend
xlabel('X (m)'); ylabel('Y (m)');
legend;


% Extract x, y coordinates from all stored waypoints
x = waypoint_path(:,1);
y = waypoint_path(:,2);

% Plot the full path with all waypoints
figure;
plot(x, y, 'bo-'); % Blue circles for waypoints, line connecting them
hold on;
grid on;
xlabel('X');
ylabel('Y');
title('Progressively Stored Dubins Path Waypoints');
legend('Waypoints');


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

function [departure_dir, arrival_dir] = calculate_directions(coord1, coord2)
    % Calculate direction vector between two coordiantes
    direction_vector = coord2 - coord1;
    
    % Determine departure direction
    departure_dir = atan2(direction_vector(2), direction_vector(1));
    % Determine arrival direction (opposite of departure)
    arrival_dir = atan2(direction_vector(2), direction_vector(1));
end

function [image_coordinates, waypoints] = smooth_coordinate_path(coordinate_path, waypoint_path, num_waypoints_per_coord)
    % Iterates through the calculated path, and smooths out any points
    % which could reduce the energy cost
    
    num_coordinates = size(coordinate_path, 1);
    num_waypoints_total = size(waypoint_path, 1);

    % Initialise Dubins path algorithm
    connectionObj = uavDubinsConnection;
    connectionObj.MaxRollAngle = 1.1;
    % Direction UAV arrived at the previous square corner
    prev_arrival_dir = 0;

    index = 1;
    for i = 1:num_waypoints_total
        if ~mod(i, 20) || i == 1

            if (index + 2) > num_coordinates
                break
            end
            current_coord = coordinate_path(index, :);
            next_next_coord = coordinate_path(index + 2, :);

            % Calculate path from coord at index, to index + 2
            % Pass through optimal index + 1 coord
            [departure_dir, arrival_dir] = calculate_directions(current_coord, next_next_coord);

            start_pose = [current_coord, 0, prev_arrival_dir];
            goal_pose = [next_next_coord, 0, arrival_dir];
            prev_arrival_dir = arrival_dir;
            
            % Calculate Dubins Path and display on plot
            [pathSegObj, pathCosts] = connect(connectionObj, start_pose, goal_pose);
            show(pathSegObj{1})


            index = index + 2;
        end
    end


    image_coordinates = coordinate_path;
    waypoints = waypoint_path;


end


function [image_coordinates, waypoints] = smooth_coordinate_path_v2(coordinate_path, waypoint_path, num_waypoints_per_coord)
    % Smooths the calculated path by skipping unnecessary waypoints if a 
    % lower-energy Dubins path exists.

    num_coordinates = size(coordinate_path, 1);
    optimized_path = [coordinate_path(1, :)];  % Start position remains the same

    % Initialise Dubins path algorithm
    connectionObj = uavDubinsConnection;
    connectionObj.MaxRollAngle = 1.1;
    
    % Previous arrival direction
    prev_arrival_dir = 0;
    i = 1;
    waypoints = [];

    while i < num_coordinates
        current_coord = coordinate_path(i, :);
        
        % Try skipping one or more waypoints to reduce path cost
        best_next_index = i + 1; % Default next step
        min_cost = inf;
        best_segment = [];

        for j = i+1:min(i+3, num_coordinates)  % Look ahead up to 3 waypoints
            next_coord = coordinate_path(j, :);
            
            % Calculate Dubins path cost from current to next_coord
            [departure_dir, arrival_dir] = calculate_directions(current_coord, next_coord);
            start_pose = [current_coord, 0, prev_arrival_dir];
            goal_pose = [next_coord, 0, arrival_dir];
            prev_arrival_dir = arrival_dir;

            
            % Compute Dubins path
            [pathSegObj, pathCosts] = connect(connectionObj, start_pose, goal_pose);
            
            if pathCosts < min_cost
                min_cost = pathCosts;
                best_next_index = j;
                best_segment = pathSegObj{1};
            end
        end
        
        % Store the best waypoint
        optimized_path = [optimized_path; coordinate_path(best_next_index, :)];
        
        % Generate and store waypoints for the smoothed segment
        segment_waypoints = interpolate(best_segment, linspace(0, best_segment.Length, num_waypoints_per_coord));
        waypoints = [waypoints; segment_waypoints];

        % Move to the selected next coordinate
        i = best_next_index;
    end

    % Output optimized coordinates
    image_coordinates = optimized_path;
end




    % for i = 1:num_coordinates
    %     current_coord = coordinate_path(i);
    %     for j = 1:num_waypoints_per_coord
    %         current_waypoint = waypoint_path(j, 1:2);
    % 
    %         if ~mod(j, 20)
    % 
    %         end
    %     end
    % end


% % Initialise Dubins path algorithm
% connectionObj = uavDubinsConnection;
% connectionObj.MaxRollAngle = 1.1;
% % Direction UAV arrived at the previous square corner
% prev_arrival_dir = 0;
% temp_start_pose = coordinate_path(i, :);
% temp_goal_pose = coordinate_path(i+2, :);
% % Calculate start and goal position for dubins connection
% start_pose = [temp_start_pose, 0, prev_arrival_dir];
% goal_pose = [temp_goal_pose, 0, arrival_dir];
% prev_arrival_dir = arrival_dir;
% % Calculate Dubins Path and display on plot
% [pathSegObj, pathCosts] = connect(connectionObj, start_pose, goal_pose);
% show(pathSegObj{1})
% 
% dubinsPath = pathSegObj{1};
% segment_waypoints = interpolate(dubinsPath, linspace(0, dubinsPath.Length, num_waypoints));
% 
% waypoints = [waypoint_path; segment_waypoints];