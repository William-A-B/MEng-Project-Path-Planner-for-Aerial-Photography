function [coordinate_path, path_cost] = solveTravellingSalesmanProblem(start_pos, goal_pos, coordinate_points, wind_direction)
%SOLVETRAVELLINGSALESMANPROBLEM Summary of this function goes here
%   Detailed explanation goes here


% --------- TSP Solver using Nearest Neighbor Approximation --------------

    path_cost = 0;

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
        % Find the next position to move to
        [minVal, minIndex] = calculate_closest_position(coordinate_points, current_pos, wind_direction);
        % k = 5;
        % candidate_indices = get_top_k_candidates(coordinate_points, current_pos, wind_direction, k);
        % [minIndex, ~] = lookahead_heurist\ic(candidate_indices, coordinate_points, current_pos, wind_direction);
        
        next_pos = coordinate_points(minIndex, :);
            
        % [dubins_path, b] = optimise_path(prev_pos, current_pos, next_pos, wind_direction);

        % dubins_path_collection = [dubins_path_collection; dubins_path];

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

    % path_cost = dubins_path_collection;
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

function candidate_indices = get_top_k_candidates(coordinate_points, current_pos, wind_direction, k)
    % Compute adjusted cost using your wind-aware function
    distances = sqrt((coordinate_points(:, 1) - current_pos(1)).^2 + (coordinate_points(:, 2) - current_pos(2)).^2);
    movement_angles = atan2(coordinate_points(:,1) - current_pos(1), coordinate_points(:,2) - current_pos(2));
    angle_diff = abs(wrapToPi(movement_angles - wind_direction));
    wind_penalty = abs(cos(angle_diff));
    adjusted_cost = distances .* (1 + wind_penalty);

    % Sort and return indices of top-k candidates
    [~, sorted_indices] = sort(adjusted_cost);
    candidate_indices = sorted_indices(1:min(k, length(sorted_indices)));
end


function cost = calculate_adjusted_cost(p1, p2, wind_direction)
    dist = norm(p2 - p1);
    angle = atan2(p2(1) - p1(1), p2(2) - p1(2));
    angle_diff = abs(wrapToPi(angle - wind_direction));
    wind_penalty = abs(cos(angle_diff));

    % Grid-alignment penalty (encourages row/column continuity)
    lateral_shift = abs(p2(1) - p1(1));  % X-direction penalty
    vertical_shift = abs(p2(2) - p1(2)); % Y-direction can be less penalized

    % Tune these weights based on your grid spacing
    locality_penalty = 0.2 * lateral_shift + 0.05 * vertical_shift;

    cost = dist * (1 + wind_penalty) + locality_penalty;
end

function [best_index, best_cost] = lookahead_heuristic(candidate_indices, coordinate_points, current_pos, wind_direction)
    best_cost = Inf;
    best_index = candidate_indices(1); % Default

    for i = 1:length(candidate_indices)
        idx1 = candidate_indices(i);
        pos1 = coordinate_points(idx1, :);
        % Remove pos1 to simulate future selection
        remaining = coordinate_points;
        square_alignment = mod(idx1, 4);
        remaining = remove_explored_coordinates(remaining, idx1, square_alignment);

        % Get 2nd-best move from pos1
        if ~isempty(remaining)
            [~, idx2] = calculate_closest_position(remaining, pos1, wind_direction);
            pos2 = remaining(idx2, :);
            cost1 = calculate_adjusted_cost(current_pos, pos1, wind_direction);
            cost2 = calculate_adjusted_cost(pos1, pos2, wind_direction);
            total_cost = cost1 + cost2;
        else
            total_cost = calculate_adjusted_cost(current_pos, pos1, wind_direction);
        end

        if total_cost < best_cost
            best_cost = total_cost;
            best_index = idx1;
        end
    end
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

