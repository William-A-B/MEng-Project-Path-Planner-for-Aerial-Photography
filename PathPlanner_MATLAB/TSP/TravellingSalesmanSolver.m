classdef TravellingSalesmanSolver
    properties
        start_pos
        goal_pos
        coordinate_waypoints
        wind_direction
        unexplored_coordinate_points
        
    end
    %% PUBLIC FUNCTIONS
    methods (Access = public)
        function obj = TravellingSalesmanSolver(start_pos, goal_pos, coordinate_waypoints, wind_direction)
            if nargin > 0
                obj.start_pos = start_pos;
                obj.goal_pos = goal_pos;
                obj.coordinate_waypoints = coordinate_waypoints;
                obj.wind_direction = wind_direction;
                obj.unexplored_coordinate_points = coordinate_waypoints;
            else
                obj.start_pos = [];
                obj.goal_pos = [];
                obj.coordinate_waypoints = [];
                obj.wind_direction = 0;
                obj.unexplored_coordinate_points = [];
            end
        end

        function [coordinate_path, path_cost] = solveTravellingSalesmanProblem(obj)
            %SOLVETRAVELLINGSALESMANPROBLEM Summary of this function goes here
            %   Detailed explanation goes here
            
            
            % --------- TSP Solver using Nearest Neighbor Approximation --------------
            
            path_cost = 0;
        
            % Number of coordinates to explore
            num_coordinates = size(obj.coordinate_waypoints, 1)/4;
            obj.unexplored_coordinate_points = obj.coordinate_waypoints;
        
            prev_pos = obj.start_pos;
            current_pos = obj.start_pos;
            next_pos = obj.start_pos;
            % Initialise path variable
            coordinate_path = zeros(num_coordinates+2, 2);
            coordinate_path(1, :) = current_pos;
        
                
            % Loop for total number of coordinates.
            for i = 1:num_coordinates
                % Find the next position to move to
                [minVal, minIndex] = calculate_closest_position(current_pos);
        
                next_pos = obj.unexplored_coordinate_points(minIndex, :);
        
                [coordinate_path, obj.unexplored_coordinate_points] = optimisation_heuristic(current_pos, next_pos, coordinate_path, i, unexplored_coordinate_points, wind_direction);
        
                % Remove the found coordinate, so it's not explored again
                obj.unexplored_coordinate_points = remove_explored_coordinates(minIndex);
            
                % Update the coordinate path with the next position
                coordinate_path(i+1, :) = next_pos;
                % Update current and previous positions
                prev_pos = current_pos;
                current_pos = next_pos;
            end
            coordinate_path(num_coordinates+2, :) = obj.goal_pos;
        
        
        end
    end
    %% PRIVATE FUNCTIONS
    methods (Access = private)

        function unexplored_coordinate_points = remove_explored_coordinates(minIndex)
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
        
        function [minVal, minIndex] = calculate_closest_position(current_pos)
        
            % Calculate distance to all coordinates from current position
            distances = sqrt((unexplored_coordinate_points(:, 1) - current_pos(1)).^2 + (unexplored_coordinate_points(:, 2) - current_pos(2)).^2);
        
            % Compute angle of movement to each candidate point
            movement_angles = atan2(unexplored_coordinate_points(:,1) - current_pos(1), unexplored_coordinate_points(:,2) - current_pos(2));
            
            % Compute absolute angular difference to wind direction
            angle_diff = abs(wrapToPi(movement_angles - wind_direction));
            
            % Weight function: penalize movement along wind, reward perpendicular movement
            wind_penalty = abs(cos(angle_diff)); % Penalizes alignment with wind
            
            % Compute adjusted cost
            adjusted_cost = distances .* (1 + wind_penalty);
            
            % Select the best next position considering wind influence
            [minVal, minIndex] = min(adjusted_cost);
        
        end
        
        function [coordinate_path, unexplored_coordinate_points] = optimisation_heuristic(current_pos, next_pos, coordinate_path, index, coordinate_waypoints, unexplored_coordinate_points, wind_direction)
            coordinate_path = coordinate_path;
            current_coord_path = coordinate_path(1:index, :);
        
            [minVal, minIndex] = calculate_closest_position(current_coord_path, next_pos, wind_direction);
            closest_coord = current_coord_path(minIndex, :);
        
            if closest_coord ~= current_pos
                % Remove minIndex to end index of coordinate_path (retracing steps)
                coordinate_path(minIndex:end, :) = 0;
                
                % Reinsert minIndex to end index of coordinate_path into
                % unexplored_coordinate_points
                for i = minIndex:index
                    unexplored_coordinate_points = reinsert_coordinates(unexplored_coordinate_points, coordinate_waypoints, i);
                end
        
                % Update coordinate_path with the new order
                
            end
        
        end
    end
end



