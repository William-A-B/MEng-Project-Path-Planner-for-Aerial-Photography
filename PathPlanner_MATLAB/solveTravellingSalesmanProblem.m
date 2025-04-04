function [coordinate_path, path_cost] = solveTravellingSalesmanProblem(start_pos, goal_pos, coordinate_points, wind_direction)
%SOLVETRAVELLINGSALESMANPROBLEM Summary of this function goes here
%   Detailed explanation goes here


% --------- TSP Solver using Nearest Neighbor Approximation --------------
    % Number of coordinates to explore
    num_coordinates = size(coordinate_points, 1)/4;

    prev_pos = inf;
    current_pos = start_pos;
    next_pos = inf;
    % Initialise path variable
    coordinate_path = zeros(num_coordinates+2, 2);
    coordinate_path(1, :) = current_pos;
    complete = 0;

    for i = 1:num_coordinates
        % distances = pdist2(current_pos, coordinate_points);
        % Calculate distance to all coordinates from current position
        distances = sqrt((coordinate_points(:, 1) - current_pos(1)).^2 + (coordinate_points(:, 2) - current_pos(2)).^2);
        % Find the minimum distance
        [minVal, minIndex] = min(distances);
        % Find the distance to the goal position from the current position
        distance_to_goal = sqrt((goal_pos(1) - current_pos(1)).^2 + (goal_pos(2) - current_pos(2)).^2);
        
        % If the distance to the goal is the shortest distance then move to
        % the goal. If not carry on exploring the grid
        if distance_to_goal < minVal
            next_pos = goal_pos;
            complete = 1;
        else
            next_pos = coordinate_points(minIndex, :);
        end
            
        % Remove the found coordinate, so it's not explored again
        square_alignment = mod(minIndex, 4);
        
        coordinate_points = remove_explored_coordinates(coordinate_points, minIndex, square_alignment);
    
        % Update the coordinate path with the next position
        coordinate_path(i+1, :) = next_pos;
        % Update current and previous positions
        prev_pos = current_pos;
        current_pos = next_pos;

        if complete == 1
            break;
        end
        
    end

    path_cost = 2;
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