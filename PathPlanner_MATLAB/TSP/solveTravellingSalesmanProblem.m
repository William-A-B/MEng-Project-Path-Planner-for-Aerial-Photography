function [coordinate_path, dubins_path_collection] = solveTravellingSalesmanProblem(start_pos, goal_pos, coordinate_waypoints, wind_direction)
%SOLVETRAVELLINGSALESMANPROBLEM Summary of this function goes here
%   Detailed explanation goes here


% --------- TSP Solver using Nearest Neighbor Approximation --------------

    dubins_path_collection = [];

    % Number of coordinates to explore
    num_coordinates = size(coordinate_waypoints, 1)/4;
    unexplored_coordinate_points = coordinate_waypoints;
    % Initialise path variable
    coordinate_path = zeros(num_coordinates+2, 2);
    coordinate_path(1, :) = start_pos;

    [starting_coordinate, start_coord_index] = update_start_pos_with_wind_compensation(start_pos, wind_direction, coordinate_waypoints);

    prev_pos = starting_coordinate;
    current_pos = starting_coordinate;
    next_pos = starting_coordinate;

    coordinate_path(2, :) = current_pos;
    unexplored_coordinate_points = remove_explored_coordinates(unexplored_coordinate_points, start_coord_index);

    loop_index = 2;
    loop_count = 1;
    %% Loop for total number of coordinates.
    while loop_index <= num_coordinates
        % disp(loop_count);
        
        if isempty(unexplored_coordinate_points)
            break;
        end

        % Find the next position to move to
        [minVal, next_pos_index] = calculate_closest_position(unexplored_coordinate_points, current_pos, wind_direction, true);

        next_pos = unexplored_coordinate_points(next_pos_index, :);
        
        next_pos_path_midpoint = calculate_midpoint(current_pos, next_pos);

        [minVal, midpoint_index] = calculate_closest_position(unexplored_coordinate_points, next_pos_path_midpoint, wind_direction, false);
        nearest_coord_midpoint = unexplored_coordinate_points(midpoint_index, :);
        if nearest_coord_midpoint ~= next_pos
            next_pos = nearest_coord_midpoint;
            next_pos_index = midpoint_index;
        end

        % Calculate dubins path between coordinates
        % Dubins path doesn't account for wind direction, it uses the basis
        % of the next_pos calculated above which does use the wind, but
        % then since the dubins path tests all 4 imaging spots per square
        % in the optimised function, then it doesn't account for wind. This
        % can be implemented by doing something similar to
        % calculate_closest_position and adding the wind penalty onto the
        % dubins path cost value.
        % [dubins_path_segment, dubins_path_cost] = calculate_dubins_connection(prev_pos, current_pos, next_pos);
        [dubins_path_segment, dubins_path_cost, new_next_pos] = calculate_optimised_dubins_connection(unexplored_coordinate_points, prev_pos, current_pos, next_pos);
        
        % update next position if the dubins path new position is different
        if ~isequal(next_pos, new_next_pos)
            next_pos = new_next_pos;
            [next_pos_index, ~] = find(ismember(unexplored_coordinate_points, next_pos, 'rows'));
        end
        
        % Remove the found coordinate, so it's not explored again
        unexplored_coordinate_points = remove_explored_coordinates(unexplored_coordinate_points, next_pos_index);

        % Heuristics optimisation, old, currently not working, look into
        % fixing
        % if size(coordinate_path(1:loop_index, :), 1) > 1
        %     [coordinate_path, unexplored_coordinate_points, reset_index] = optimisation_heuristic(current_pos, next_pos, coordinate_path, loop_index, coordinate_waypoints, unexplored_coordinate_points, wind_direction);
        %     loop_index = reset_index;
        % end
    
        % Update the coordinate path with the next position
        coordinate_path(loop_index+1, :) = next_pos;
        
        % Update current and previous positions
        prev_pos = current_pos;
        current_pos = next_pos;
        dubins_path_collection = [dubins_path_collection; dubins_path_segment];

        loop_index = loop_index + 1;
        loop_count = loop_count + 1;

        % plot(coordinate_path(1:loop_index,1), coordinate_path(1:loop_index,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');
    end
    coordinate_path(num_coordinates+2, :) = goal_pos;

end

function unexplored_coordinate_points = remove_explored_coordinates(unexplored_coordinate_points, minIndex)
    square_alignment = mod(minIndex, 4);
    switch square_alignment
        case 0
            unexplored_coordinate_points(minIndex-3:minIndex, :) = [];
        case 1
            unexplored_coordinate_points(minIndex:minIndex+3, :) = [];
        case 2
            unexplored_coordinate_points(minIndex-1:minIndex+2, :) = [];
        case 3
            unexplored_coordinate_points(minIndex-2:minIndex+1, :) = [];
        otherwise
            return
    end
end

function imaging_area_coordinates = get_image_area_coordinates(coordinate_waypoints, index)
    square_alignment = mod(index, 4);
    switch square_alignment
        case 0
            imaging_area_coordinates = coordinate_waypoints(index-3:index, :);
        case 1
            imaging_area_coordinates = coordinate_waypoints(index:index+3, :);
        case 2
            imaging_area_coordinates = coordinate_waypoints(index-1:index+2, :);
        case 3
            imaging_area_coordinates = coordinate_waypoints(index-2:index+1, :);
        otherwise
            return
    end
end

function unexplored_coordinate_points = reinsert_coordinates(unexplored_coordinate_points, coordinate_waypoints, index)
    square_alignment = mod(index, 4);
    switch square_alignment
        case 0
            unexplored_coordinate_points = [unexplored_coordinate_points; coordinate_waypoints(index-3:index, :)];
        case 1
            unexplored_coordinate_points = [unexplored_coordinate_points; coordinate_waypoints(index:index+3, :)];
        case 2
            unexplored_coordinate_points = [unexplored_coordinate_points; coordinate_waypoints(index-1:index+2, :)];
        case 3
            unexplored_coordinate_points = [unexplored_coordinate_points; coordinate_waypoints(index-2:index+1, :)];
        otherwise
            return
    end
end

function [minVal, minIndex] = calculate_closest_position(coordinate_points, current_pos, wind_direction, apply_wind_compensation)

    % Calculate distance to all coordinates from current position
    distances = sqrt((coordinate_points(:, 1) - current_pos(1)).^2 + (coordinate_points(:, 2) - current_pos(2)).^2);

    % Compute angle of movement to each candidate point
    movement_angles = atan2(coordinate_points(:,1) - current_pos(1), coordinate_points(:,2) - current_pos(2));
    
    % Compute absolute angular difference to wind direction
    angle_diff = abs(wrapToPi(movement_angles - wind_direction));
    
    % Weight function: penalize movement along wind, reward perpendicular movement
    wind_penalty = abs(cos(angle_diff)); % Penalizes alignment with wind
    
    % Compute adjusted cost
    if apply_wind_compensation
        adjusted_cost = distances .* (1 + (wind_penalty*6));
    else
        adjusted_cost = distances .* (1 + (wind_penalty*4));
    end
    % Select the best next position considering wind influence
    [minVal, minIndex] = min(adjusted_cost);

end

function [coordinate_path, unexplored_coordinate_points, reset_index] = optimisation_heuristic(current_pos, next_pos, coordinate_path, index, coordinate_waypoints, unexplored_coordinate_points, wind_direction)
    current_coord_path = coordinate_path(1:index, :);
    reset_index = index;

    [minVal, minIndex] = calculate_closest_position(current_coord_path, next_pos, wind_direction, true);
    closest_coord = current_coord_path(minIndex, :);

    if closest_coord ~= current_pos
        % Remove minIndex to end index of coordinate_path (retracing steps)
        coordinate_path(minIndex+1:end, :) = 0;
      
        % Reinsert minIndex to end index of coordinate_path into
        % unexplored_coordinate_points
        for i = minIndex+1:index
            unexplored_coordinate_points = reinsert_coordinates(unexplored_coordinate_points, coordinate_waypoints, i);
        end

        % Update coordinate_path with the new order
        reset_index = minIndex;
    end
    
end


function [starting_coordinate, start_coord_index] = update_start_pos_with_wind_compensation(start_pos, wind_direction, coordinate_waypoints)
    % Calculate wind vector
    wind_dx = sin(wind_direction);
    wind_dy = cos(wind_direction);

    
    % Vector from start pos to all points
    Vx = coordinate_waypoints(:, 1) - start_pos(:, 1);
    Vy = coordinate_waypoints(:, 2) - start_pos(:, 2);
    
    % Project coordinate points onto wind direction vector
    projection = (Vx * wind_dx) + (Vy * wind_dy);
    
    % Calculate furthest point downwind
    [downwind_weight, downwind_coord_index] = max(projection);

    downwind_coord = coordinate_waypoints(downwind_coord_index, :);

    starting_coordinate = downwind_coord;
    start_coord_index = downwind_coord_index;

end

function midpoint = calculate_midpoint(pos1, pos2)
    midpoint = [0, 0];
    midpoint(1) = (pos1(1) + pos2(1)) / 2;
    midpoint(2) = (pos1(2) + pos2(2)) / 2;
end

function bearing = calculate_bearing(pos1, pos2)
    bearing = 0;

    bearing = atan2(pos1(1) - pos2(1), pos1(2) - pos2(2));
end

function [departure_dir, arrival_dir] = calculate_directions(coord1, coord2)
    % Calculate direction vector between two coordiantes
    direction_vector = coord2 - coord1;
    
    % Determine departure direction
    departure_dir = atan2(direction_vector(2), direction_vector(1));
    % Determine arrival direction (opposite of departure)
    arrival_dir = atan2(direction_vector(2), direction_vector(1));
end

function [path_segment, path_cost] = calculate_dubins_connection(previous_pos, current_pos, next_pos)
    dubConnObj = dubinsConnection;
    dubConnObj.MinTurningRadius = 30;

    [~, prev_arrival_dir] = calculate_directions(previous_pos, current_pos);
    [~, next_arrival_dir] = calculate_directions(current_pos, next_pos);

    start_pose = [current_pos, prev_arrival_dir];
    goal_pose = [next_pos, next_arrival_dir];

    [pathSegObj, pathCosts] = connect(dubConnObj, start_pose, goal_pose);
    
    path_segment = pathSegObj;
    path_cost = pathCosts;
end

function [path_segment, path_cost, new_next_pos] = calculate_optimised_dubins_connection(coordinate_waypoints, previous_pos, current_pos, next_pos)
    dubConnObj = dubinsConnection;
    dubConnObj.MinTurningRadius = 4;

    [next_pos_index, ~] = find(ismember(coordinate_waypoints, next_pos, 'rows'));
    possible_next_coordinates = get_image_area_coordinates(coordinate_waypoints, next_pos_index);

    [~, prev_arrival_dir] = calculate_directions(previous_pos, current_pos);

    path_cost = inf;
    new_next_pos = 0;

    for i = 1:size(possible_next_coordinates, 1)
        current_next_pos = possible_next_coordinates(i, :);
        [~, next_arrival_dir] = calculate_directions(current_pos, current_next_pos);

        start_pose = [current_pos, prev_arrival_dir];
        goal_pose = [current_next_pos, next_arrival_dir];

        [pathSegObj, current_path_cost] = connect(dubConnObj, start_pose, goal_pose);

        if current_path_cost < path_cost
            path_cost = current_path_cost;
            path_segment = pathSegObj;
            new_next_pos = current_next_pos;
        end
    end
end