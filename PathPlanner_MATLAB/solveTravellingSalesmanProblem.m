function [coordinate_path, path_cost] = solveTravellingSalesmanProblem(start_pos, goal_pos, coordinate_points, wind_direction)
%SOLVETRAVELLINGSALESMANPROBLEM Summary of this function goes here
%   Detailed explanation goes here


% --------- TSP Solver using Nearest Neighbor Approximation --------------
    % Number of coordinates to explore
    num_coordinates = size(coordinate_points, 1)/4;

    prev_pos = start_pos;
    current_pos = start_pos;
    next_pos = start_pos;
    % Initialise path variable
    coordinate_path = zeros(num_coordinates+2, 2);
    coordinate_path(1, :) = current_pos;

    waypoint_path = [];
    num_waypoints_per_coord = 20;

    dubins_path_collection = [];

    % Initialise Dubins path algorithm
    connectionObj = uavDubinsConnection;
    connectionObj.MaxRollAngle = 1;
        
    %% Loop for total number of coordinates.
    for i = 1:num_coordinates
        % distances = pdist2(current_pos, coordinate_points);
        % Find the next position to move to
        [minVal, minIndex] = calculate_closest_position(coordinate_points, current_pos, wind_direction);
        % Find the distance to the goal position from the current position
        distance_to_goal = sqrt((goal_pos(1) - current_pos(1)).^2 + (goal_pos(2) - current_pos(2)).^2);
        
        next_pos = coordinate_points(minIndex, :);
            
        [dubins_path, b] = optimise_path(prev_pos, current_pos, next_pos, wind_direction);

        dubins_path_collection = [dubins_path_collection; dubins_path];

        % Remove the found coordinate, so it's not explored again
        square_alignment = mod(minIndex, 4);
        coordinate_points = remove_explored_coordinates(coordinate_points, minIndex, square_alignment);
    
        % Update the coordinate path with the next position
        coordinate_path(i+1, :) = next_pos;
        % Update current and previous positions
        prev_pos = current_pos;
        current_pos = next_pos;
        
    end
    
    coordinate_path(num_coordinates+2, :) = goal_pos;

    path_cost = dubins_path_collection;
end

function coordinate_points = remove_explored_coordinates(coordinate_points, minIndex, square_alignment)
    switch square_alignment
        case 0
            coordinate_points(minIndex-3:minIndex, :) = [];
        case 1
            coordinate_points(minIndex:minIndex+3, :) = [];
        case 2
            coordinate_points(minIndex-1:minIndex+2, :) = [];
        case 3
            coordinate_points(minIndex-2:minIndex+1, :) = [];
        otherwise
            return
    end
end

function [minVal, minIndex] = calculate_closest_position(coordinate_points, current_pos, wind_direction)

    % Calculate distance to all coordinates from current position
    distances = sqrt((coordinate_points(:, 1) - current_pos(1)).^2 + (coordinate_points(:, 2) - current_pos(2)).^2);

    % Compute angle of movement to each candidate point
    movement_angles = atan2(coordinate_points(:,1) - current_pos(1), coordinate_points(:,2) - current_pos(2));
    
    % Compute absolute angular difference to wind direction
    angle_diff = abs(wrapToPi(movement_angles - wind_direction));
    
    % Weight function: penalize movement along wind, reward perpendicular movement
    wind_penalty = abs(cos(angle_diff)); % Penalizes alignment with wind
    
    % Compute adjusted cost
    adjusted_cost = distances .* (1 + wind_penalty);
    
    % Select the best next position considering wind influence
    [minVal, minIndex] = min(adjusted_cost);

end

function [optimum_dubins_path, b] = optimise_path(prev_pos, current_pos, next_pos, wind_direction)
    waypoint_path = [];
    num_waypoints_per_coord = 20;

    % Initialise Dubins path algorithm
    connectionObj = uavDubinsConnection;
    connectionObj.MaxRollAngle = 1;

    % Calculate departure and arrival directions between each square
    [departure_dir, arrival_dir] = calculate_directions(prev_pos, next_pos);

    prev_min_distance = inf;
    min_distance = inf;
    cheapest_path_angle = departure_dir;
    current_path_departure_angle = departure_dir - pi/2;
    optimum_dubins_path = 0;
    
    % Loop across 180 view, in 5 degree increments
    for i = 1:36
        % Calculate start and goal position for dubins connection
        start_pose = [prev_pos, 0, current_path_departure_angle];
        goal_pose = [next_pos, 0, arrival_dir];

         % Calculate Dubins Path
        [pathSegObj, pathCosts] = connect(connectionObj, start_pose, goal_pose);
    
        dubinsPath = pathSegObj{1};
        segment_waypoints = interpolate(dubinsPath, linspace(0, dubinsPath.Length, num_waypoints_per_coord));
    
        min_distance = find_min_distance(segment_waypoints, current_pos);
    
        if min_distance < prev_min_distance
            cheapest_path_angle = current_path_departure_angle;
            optimum_dubins_path = dubinsPath;
        end
        current_path_departure_angle = current_path_departure_angle + 0.0873;
        prev_min_distance = min_distance;
    end

    % Calculate start and goal position for dubins connection
    start_pose = [prev_pos, 0, cheapest_path_angle];
    goal_pose = [next_pos, 0, arrival_dir];
    
    % Do something here to connect up the previous and next by going
    % through the current pos

    b = 1;
end

function [departure_dir, arrival_dir] = calculate_directions(coord1, coord2)
    % Calculate direction vector between two coordiantes
    direction_vector = coord2 - coord1;
    
    % Determine departure direction
    departure_dir = atan2(direction_vector(2), direction_vector(1));
    % Determine arrival direction (opposite of departure)
    arrival_dir = atan2(direction_vector(2), direction_vector(1));
end

function min_distance = find_min_distance(coordinate_points, current_pos)
    % Calculate distance to all coordinates from current position
    distances = sqrt((coordinate_points(:, 1) - current_pos(1)).^2 + (coordinate_points(:, 2) - current_pos(2)).^2);
    % Find minimum
    min_distance = min(distances);
end