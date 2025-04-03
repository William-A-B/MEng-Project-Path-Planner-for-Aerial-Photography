function [coordinate_path, path_cost] = solveTravellingSalesmanProblem(start_pos, goal_pos, coordinate_points, wind_direction)
%SOLVETRAVELLINGSALESMANPROBLEM Summary of this function goes here
%   Detailed explanation goes here


% --------- TSP Solver using Nearest Neighbor Approximation --------------
    
    num_coordinates = size(coordinate_points, 1)/4;

    prev_pos = inf;
    current_pos = start_pos;
    next_pos = inf;
    coordinate_path = zeros(num_coordinates+2, 2);
    coordinate_path(1, :) = current_pos;

    for i = 1:num_coordinates
        % distances = pdist2(current_pos, coordinate_points);
        distances = sqrt((coordinate_points(:, 1) - current_pos(1)).^2 + (coordinate_points(:, 2) - current_pos(2)).^2);
        [minVal, minIndex] = min(distances);
        distance_to_goal = sqrt((goal_pos(1) - current_pos(1)).^2 + (goal_pos(2) - current_pos(2)).^2);
        
        if distance_to_goal < minVal
            next_pos = goal_pos;
        else
            next_pos = coordinate_points(minIndex, :);
        end
            
        coordinate_points(minIndex, :) = [];

        coordinate_path(i+1, :) = next_pos;
        prev_pos = current_pos;
        current_pos = next_pos;


        
    end

    path_cost = 2;
end

