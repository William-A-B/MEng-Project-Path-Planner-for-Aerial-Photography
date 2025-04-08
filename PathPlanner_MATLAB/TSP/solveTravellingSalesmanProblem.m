function [coordinate_path, path_cost] = solveTravellingSalesmanProblem(start_pos, goal_pos, coordinate_waypoints, wind_direction)
%SOLVETRAVELLINGSALESMANPROBLEM Summary of this function goes here
%   Detailed explanation goes here


% --------- TSP Solver using Nearest Neighbor Approximation --------------

    path_cost = 0;

    % Number of coordinates to explore
    num_coordinates = size(coordinate_waypoints, 1)/4;
    unexplored_coordinate_points = coordinate_waypoints;

    prev_pos = start_pos;
    current_pos = start_pos;
    next_pos = start_pos;
    % Initialise path variable
    coordinate_path = zeros(num_coordinates+2, 2);
    coordinate_path(1, :) = current_pos;

    loop_index = 1;
    loop_count = 1;
    %% Loop for total number of coordinates.
    while loop_index <= num_coordinates
        disp(loop_count)
        
        if isempty(unexplored_coordinate_points)
            break;
        end

        % Find the next position to move to
        [minVal, minIndex] = calculate_closest_position(unexplored_coordinate_points, current_pos, wind_direction);

        next_pos = unexplored_coordinate_points(minIndex, :);
        % Remove the found coordinate, so it's not explored again
        unexplored_coordinate_points = remove_explored_coordinates(unexplored_coordinate_points, minIndex);

        

        if size(coordinate_path(1:loop_index, :), 1) > 1
            [coordinate_path, unexplored_coordinate_points, reset_index] = optimisation_heuristic(current_pos, next_pos, coordinate_path, loop_index, coordinate_waypoints, unexplored_coordinate_points, wind_direction);
            loop_index = reset_index;
        end
    
        % Update the coordinate path with the next position
        coordinate_path(loop_index+1, :) = next_pos;
        
        % Update current and previous positions
        prev_pos = current_pos;
        current_pos = next_pos;

        loop_index = loop_index + 1;
        loop_count = loop_count + 1;

        plot(coordinate_path(1:loop_index,1), coordinate_path(1:loop_index,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');
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

function [coordinate_path, unexplored_coordinate_points, reset_index] = optimisation_heuristic(current_pos, next_pos, coordinate_path, index, coordinate_waypoints, unexplored_coordinate_points, wind_direction)
    current_coord_path = coordinate_path(1:index, :);
    reset_index = index;

    [minVal, minIndex] = calculate_closest_position(current_coord_path, next_pos, wind_direction);
    current_pos
    next_pos
    closest_coord = current_coord_path(minIndex, :)

    if closest_coord ~= current_pos
        % Remove minIndex to end index of coordinate_path (retracing steps)
        coordinate_path(minIndex+1:end, :) = 0
      
        % Reinsert minIndex to end index of coordinate_path into
        % unexplored_coordinate_points
        for i = minIndex+1:index
            unexplored_coordinate_points = reinsert_coordinates(unexplored_coordinate_points, coordinate_waypoints, i);
        end

        % Update coordinate_path with the new order
        reset_index = minIndex;
    end
    
end