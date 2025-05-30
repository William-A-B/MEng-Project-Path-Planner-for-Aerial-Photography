function [coordinate_path, dubins_path_collection, total_path_cost] = solveTravellingSalesmanProblem(start_pos, goal_pos, coordinate_waypoints, wind_direction, uav_turning_radius, uav_airspeed, num_divisions_z)
%SOLVETRAVELLINGSALESMANPROBLEM Solves a TSP problem with Dubins path constraints and wind-aware navigation.
% 
%   [coordinate_path, dubins_path_collection] = solveTravellingSalesmanProblem(start_pos, goal_pos, coordinate_waypoints, wind_direction)
%
%   This function approximates a solution to the Travelling Salesman Problem (TSP)
%   using a nearest neighbour heuristic while accounting for wind direction
%   and calculating Dubins path routes.
%   Each waypoint is part of a 2x2 imaging grid (4 points per square), and
%   only one point per grid is selected in the final path.
%   
%   Inputs:
%       start_pos              - 1x2 vector representing the starting [x, y] position
%       goal_pos               - 1x2 vector representing the goal [x, y] position
%       coordinate_waypoints   - Nx2 matrix of waypoint coordinates, where N is a multiple of 4
%                                (each group of 4 points corresponds to one imaging grid)
%       wind_direction         - Scalar representing wind direction in radians (measured clockwise from the y-axis)
%       uav_turning_radius     - The minimum turning radius of the UAV in metres
%       num_divisions_z        - The number of altitudes per imaging coordinate
%
%   Outputs:
%       coordinate_path        - Mx2 matrix of coordinates forming the TSP path, including start and goal
%       dubins_path_collection - Collection of Dubins path segments connecting each pair of consecutive points in coordinate_path
%
%   Notes:
%       - This algorithm includes wind-aware cost adjustments when choosing the
%         next waypoint, penalizing travel aligned with the wind direction.
%       - Uses Dubins paths (non-holonomic constraints) for connecting waypoints.
%       - Each 2x2 imaging area contributes only one point to the final path.
%
%   Dependencies:
%       - Requires Robotics System Toolbox for 'dubinsConnection'
%       - Auxiliary functions: calculate_closest_position, remove_explored_coordinates,
%         calculate_dubins_connection, calculate_optimised_dubins_connection, etc.


% --------- TSP Solver using Nearest Neighbor Approximation --------------

    % coordinate_waypoints_3d = setup_elevation_data(coordinate_waypoints);
    % start_pos = [start_pos, 0];
    % goal_pos = [goal_pos, 0];

    global num_div_z;
    num_div_z = num_divisions_z;

    num_2d_coordinates = size(coordinate_waypoints, 1) / 3;

    start_pos_2d = start_pos(1, 1:2);
    goal_pos_2d = goal_pos(1, 1:2);
    coordinate_waypoints_2d = coordinate_waypoints(1:num_2d_coordinates, 1:2);

    start_pos = start_pos;
    goal_pos = goal_pos;
    coordinate_waypoints = coordinate_waypoints;

    dubins_path_collection = [];
    total_path_cost = 0;

    % Number of coordinates to explore
    num_imaging_locations = size(coordinate_waypoints, 1)/(4*num_div_z);
    unexplored_coordinate_points = coordinate_waypoints;
    % Initialise path variable
    coordinate_path = zeros(num_imaging_locations+2, 3);
    coordinate_path(1, :) = start_pos;

    [starting_coordinate, start_coord_index] = update_start_pos_with_wind_compensation(start_pos, wind_direction, coordinate_waypoints);

    prev_pos = start_pos;
    current_pos = start_pos;
    next_pos = starting_coordinate;

    [dubins_path_segment, dubins_path_cost] = calculate_dubins_connection_3d(prev_pos, current_pos, next_pos, uav_turning_radius, uav_airspeed);

    coordinate_path(2, :) = next_pos;
    unexplored_coordinate_points = remove_explored_coordinates_3d(unexplored_coordinate_points, start_coord_index);
    dubins_path_collection = [dubins_path_collection; dubins_path_segment];

    % Update current and previous positions
    prev_pos = current_pos;
    current_pos = next_pos;

    total_path_cost = total_path_cost + dubins_path_cost;

    loop_index = 2;
    loop_count = 1;
    %% Loop for total number of coordinates.
    while loop_index <= num_imaging_locations
        
        if isempty(unexplored_coordinate_points)
            break;
        end

        % Find the next position to move to
        % [~, next_pos_index] = calculate_closest_position(unexplored_coordinate_points, current_pos, wind_direction, true);
        % [~, next_pos_index] = calculate_closest_position_3d(unexplored_coordinate_points, current_pos, wind_direction, true);
        [next_pos_coord, next_pos_index, next_pos_path_cost] = calculate_closest_position_3d_heuristics(unexplored_coordinate_points, current_pos, wind_direction, true);


        next_pos = unexplored_coordinate_points(next_pos_index, :);
        % Calculate the midpoint between the current coordinate and next
        % coordinate
        next_pos_path_midpoint = calculate_midpoint(current_pos, next_pos);
        % Check if there is a closer coordinate to the midpoint than the
        % current destination (next_pos), if there is re-route and update
        % next_pos with the new closest position
        [~, midpoint_index] = calculate_closest_position_3d_heuristics(unexplored_coordinate_points, next_pos_path_midpoint, wind_direction, false);
        nearest_coord_midpoint = unexplored_coordinate_points(midpoint_index, :);
        if ~isequal(next_pos, nearest_coord_midpoint)
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
        [dubins_path_segment, dubins_path_cost, new_next_pos] = calculate_optimised_dubins_connection_3d(unexplored_coordinate_points, prev_pos, current_pos, next_pos, uav_turning_radius, uav_airspeed);

        % update next position if the dubins path new position is different
        if ~isequal(next_pos, new_next_pos)
            next_pos = new_next_pos;
            [next_pos_index, ~] = find(ismember(unexplored_coordinate_points, next_pos, 'rows'));
        end
        
        % Remove the found coordinate, so it's not explored again
        unexplored_coordinate_points = remove_explored_coordinates_3d(unexplored_coordinate_points, next_pos_index);
    
        % Update the coordinate path with the next position
        coordinate_path(loop_index+1, :) = next_pos;
        
        % Update current and previous positions
        prev_pos = current_pos;
        current_pos = next_pos;
        dubins_path_collection = [dubins_path_collection; dubins_path_segment];
        total_path_cost = total_path_cost + dubins_path_cost;

        loop_index = loop_index + 1;
        loop_count = loop_count + 1;

        % plot(coordinate_path(1:loop_index,1), coordinate_path(1:loop_index,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');
    end
    % Explored all grid coordinate, calculate dubins path from final
    % coordinate to the goal position.
    [dubins_path_segment, dubins_path_cost] = calculate_dubins_connection_3d(prev_pos, current_pos, goal_pos, uav_turning_radius, uav_airspeed);
    coordinate_path(num_imaging_locations+2, :) = goal_pos;
    dubins_path_collection = [dubins_path_collection; dubins_path_segment];
    total_path_cost = total_path_cost + dubins_path_cost;
end

function unexplored_coordinate_points = remove_explored_coordinates(unexplored_coordinate_points, minIndex)
    %REMOVE_EXPLORED_COORDINATES Removes all 4 points of a selected 2x2 grid
    % from the list of unexplored coordinates based on the selected index.
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

function unexplored_coordinate_points = remove_explored_coordinates_3d(unexplored_coordinate_points, minIndex)
    %REMOVE_EXPLORED_COORDINATES_3D Removes all corner points of a selected
    % cuboid from the list of unexplored coordinates based on the selected index.
    global num_div_z;
    num_points_per_cuboid = num_div_z * 4;
    cuboid_alignment = mod((minIndex-1), num_points_per_cuboid);
    cuboid_start_index = minIndex - cuboid_alignment;
    cuboid_end_index = cuboid_start_index + (num_points_per_cuboid - 1);

    % Don't delete anything out of bounds
    if cuboid_start_index < 1 || cuboid_end_index > size(unexplored_coordinate_points, 1)
        warning('Attempting to remove points outside array bounds.');
        return;
    end
    % Remove the full cuboid block
    unexplored_coordinate_points(cuboid_start_index:cuboid_end_index, :) = [];
end

function imaging_area_coordinates = get_image_area_coordinates(coordinate_waypoints, index)
    %GET_IMAGE_AREA_COORDINATES Retrieves all 4 points of the 2x2 imaging grid 
    % associated with a given index from the full coordinate list.
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

function imaging_area_coordinates = get_image_area_coordinates_3d(coordinate_waypoints, index)
    %GET_IMAGE_AREA_COORDINATES_3D Retrieves all corner points of a selected
    % cuboid associated with a given index from the full coordinate list.
    global num_div_z;
    num_points_per_cuboid = num_div_z * 4;
    cuboid_alignment = mod((index-1), num_points_per_cuboid);
    cuboid_start_index = index - cuboid_alignment;
    cuboid_end_index = cuboid_start_index + (num_points_per_cuboid - 1);

    % Don't delete anything out of bounds
    if cuboid_start_index < 1 || cuboid_end_index > size(coordinate_waypoints, 1)
        warning('Attempting to retrieve points outside array bounds.');
        return;
    end
    % Remove the full cuboid block
    imaging_area_coordinates = coordinate_waypoints(cuboid_start_index:cuboid_end_index, :);
end

function [minVal, minIndex] = calculate_closest_position(coordinate_points, current_pos, wind_direction, apply_wind_compensation)
    %CALCULATE_CLOSEST_POSITION Finds the nearest coordinate considering both 
    % Euclidean distance and wind influence. Returns index and adjusted cost.

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

function [minVal, minIndex] = calculate_closest_position_3d(coordinate_points, current_pos, wind_direction, apply_wind_compensation)
    %CALCULATE_CLOSEST_POSITION Finds the nearest coordinate considering both 
    % Euclidean distance and wind influence. Returns index and adjusted cost.

    global num_div_z;
    num_cuboid_points = 4*num_div_z;

    % Extract the 3rd column (elevation) from the coordinate points
    elevations = coordinate_points(:, 3);
        
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
    
    % Take top 10 smallest distance and find the one with lowest change in
    % altitude
    % Get the top 10 smallest distances and their corresponding indices
    [~, sorted_indices] = sort(adjusted_cost);  % Sort the distances in ascending order
    if size(sorted_indices, 1) < num_cuboid_points
        closest_indices = sorted_indices(1:end);    % Get indices of top smallest distances
    else
        closest_indices = sorted_indices(1:num_cuboid_points);    % Get indices of top num_cuboid_points smallest distances
    end

    % Extract the top 10 closest coordinates and elevations
    closest_coords = coordinate_points(closest_indices, :);
    closest_points_elevations = elevations(closest_indices);

    % Calculate the change in elevation for each of the top 10 closest points
    elevation_changes = abs(closest_points_elevations - current_pos(3));

    % Find the index of the smallest elevation change among the top 10
    [~, min_elevation_index] = min(elevation_changes);

    % Select the coordinate with the smallest elevation change
    minIndex = closest_indices(min_elevation_index);
    minVal = adjusted_cost(minIndex);  % Return the adjusted cost for that coordinate
end

function [closest_coord, closest_coord_index, uav_path_cost] = calculate_closest_position_3d_improved(coordinate_points, current_pos, wind_direction, apply_wind_comp)
    uav_cost = inf;
    lowest_cost_coord_index = 1;
    loop_index = 1;
    
    while loop_index <= size(coordinate_points, 1)
        target_pos = coordinate_points(loop_index, :);
        
        current_uav_cost = calculate_flight_path_cost(current_pos, target_pos, wind_direction, apply_wind_comp);
    
        if current_uav_cost < uav_cost
            uav_cost = current_uav_cost;
            [new_target_pos_index, ~] = find(ismember(coordinate_points, target_pos, 'rows'));
            lowest_cost_coord_index = new_target_pos_index; 
        end
        loop_index = loop_index + 1;
    end
    closest_coord_index = lowest_cost_coord_index;
    closest_coord = coordinate_points(closest_coord_index, :);
    uav_path_cost = uav_cost;
end

function [closest_coord, closest_coord_index, uav_path_cost] = calculate_closest_position_3d_heuristics(coordinate_points, current_pos, wind_direction, apply_wind_comp)
    uav_cost = inf;
    lowest_cost_coord_index = 1;
    loop_index = 1;
    
    while loop_index <= size(coordinate_points, 1)
        target_pos = coordinate_points(loop_index, :);
        
        current_uav_cost = calculate_flight_path_cost(current_pos, target_pos, wind_direction, apply_wind_comp);
    
        if current_uav_cost < uav_cost
            uav_cost = current_uav_cost;
            lowest_cost_coord = target_pos;
            lowest_cost_coord_index = loop_index; 
        end
        loop_index = loop_index + 1;
    end

    % Apply heuristics
    next_uav_cost = inf;
    loop_index = 1;
    new_current_pos = lowest_cost_coord;
    new_current_pos_index = lowest_cost_coord_index;

    % Get cuboid points of the first closest point
    closest_coord_cuboid = get_image_area_coordinates_3d(coordinate_points, new_current_pos_index);

    % Iterate through all possible second-step candidates
    for i = 1:size(coordinate_points, 1)
        % Get second target position
        second_target_pos = coordinate_points(i, :);

        % Skip if second position is equal to previous closest position or
        % same cuboid
        if ismember(closest_coord_cuboid, second_target_pos, 'rows')
            continue;
        end

        second_target_cost = calculate_flight_path_cost(new_current_pos, second_target_pos, wind_direction, apply_wind_comp);

        if second_target_cost < next_uav_cost
            next_uav_cost = second_target_cost;
            next_lowest_cost_coord_index = i;
            next_lowest_cost_coord = second_target_pos;
        end
    end
    total_original_cost = uav_cost + next_uav_cost;

    % For each cuboid neighbour of the first position, calculate the full
    % path cost
    for j = 1:size(closest_coord_cuboid, 1)
        alt_first_target_pos = closest_coord_cuboid(j, :);

        % Skip if alt_first_step == first closest_coord
        if isequal(alt_first_target_pos, new_current_pos)
            continue;
        end
        % Check cost from current -> alt_first -> potential_second
        cost1 = calculate_flight_path_cost(current_pos, alt_first_target_pos, wind_direction, apply_wind_comp);
        cost2 = calculate_flight_path_cost(alt_first_target_pos, second_target_pos, wind_direction, apply_wind_comp);
        total_heuristic_cost = cost1 + cost2;

        if total_heuristic_cost < total_original_cost
            uav_cost = cost1;
            next_uav_cost = cost2;
            lowest_cost_coord = alt_first_target_pos;
            lowest_cost_coord_index = find(ismember(coordinate_points, alt_first_target_pos, 'rows'));
        end
    end

    % Update final values
    closest_coord = lowest_cost_coord;
    closest_coord_index = lowest_cost_coord_index;
    uav_path_cost = uav_cost;
end

function uav_cost = calculate_flight_path_cost(current_pos, target_pos, wind_direction, apply_wind_comp)
    %CALCULATE_FLIGHT_PATH_COST Calculates the cost for the UAV to fly
    % between two three dimensional coordinate points.
    % Accounts for wind and applies a relative cost.

    max_climb_angle = 0.7854;
    % max_climb_angle = 1.2236;
    max_climb_rate = 10;
    global num_div_z;

    base_cost_per_metre = 1;
    descent_bonus_factor = 1.1;
    climb_penalty_factor = 2;

    % Calculate the 3D distance between points
    dx = target_pos(1) - current_pos(1);
    dy = target_pos(2) - current_pos(2);
    dz = target_pos(3) - current_pos(3);

    % Calculate slope angle from current position to target position
    slope_angle = atan2(dz, sqrt(dx.^2 + dy.^2));
    if abs(slope_angle) > max_climb_angle
        uav_cost = inf;
        return
    end

    horizontal_distance = sqrt(dx.^2 + dy.^2);

    % Calculate x-y plane direction
    % Compute angle of movement to target point
    movement_angle = atan2(dx, dy);
    
    % Compute absolute angular difference to wind direction
    
    angle_diff = abs(wrapToPi(movement_angle - wind_direction));
    
    % Weight function: penalize movement along wind, reward perpendicular movement
    wind_penalty = abs(cos(angle_diff)); % Penalizes alignment with wind

    % Calculate 3 dimensional distance between positions
    pos_distance = sqrt(dx.^2 + dy.^2 + dz.^2);
    
    % Calculate base path cost
    base_cost = pos_distance * base_cost_per_metre;

    % Calculate climb/descent cost
    if slope_angle > 0
        climb_cost = slope_angle * climb_penalty_factor * horizontal_distance;
    elseif slope_angle < 0
        climb_cost = slope_angle * descent_bonus_factor * horizontal_distance;
        climb_cost = climb_cost - (base_cost * 0.1);
    else
        climb_cost = 0;
    end
    
    % Calculate total cost
    if apply_wind_comp
        uav_cost = (base_cost + climb_cost) * (1 + (wind_penalty*6));
    else
        uav_cost = base_cost + climb_cost;
    end
end

function [starting_coordinate, start_coord_index] = update_start_pos_with_wind_compensation(start_pos, wind_direction, coordinate_waypoints)
    %UPDATE_START_POS_WITH_WIND_COMPENSATION Picks the furtest position
    % downwind from the actual start position to minimize headwind movement.

    % Calculate wind vector
    wind_dx = sin(wind_direction);
    wind_dy = cos(wind_direction);

    
    % Vector from start pos to all points
    Vx = coordinate_waypoints(:, 1) - start_pos(:, 1);
    Vy = coordinate_waypoints(:, 2) - start_pos(:, 2);
    
    % Project coordinate points onto wind direction vector
    projection = (Vx * wind_dx) + (Vy * wind_dy);
    
    % Calculate furthest point downwind
    [~, downwind_coord_index] = max(projection);

    downwind_coord = coordinate_waypoints(downwind_coord_index, :);

    starting_coordinate = downwind_coord;
    start_coord_index = downwind_coord_index;

end

function [starting_coordinate, start_coord_index] = update_start_pos_with_wind_compensation_3d(start_pos, wind_direction, coordinate_waypoints)
    %UPDATE_START_POS_WITH_WIND_COMPENSATION Picks the furtest position
    % downwind from the actual start position to minimize headwind movement.

    % Calculate wind vector
    wind_dx = sin(wind_direction);
    wind_dy = cos(wind_direction);

    
    % Vector from start pos to all points
    Vx = coordinate_waypoints(:, 1) - start_pos(:, 1);
    Vy = coordinate_waypoints(:, 2) - start_pos(:, 2);
    
    % Project coordinate points onto wind direction vector
    projection = (Vx * wind_dx) + (Vy * wind_dy);
    
    % Calculate furthest point downwind
    [~, downwind_coord_index] = max(projection);

    downwind_coord = coordinate_waypoints(downwind_coord_index, :);

    starting_coordinate = downwind_coord;
    start_coord_index = downwind_coord_index;

end

function midpoint = calculate_midpoint(pos1, pos2)
    %CALCULATE_MIDPOINT Computes the midpoint between two 2D coordinates.
    if size(pos1, 2) > 2
        midpoint = [0, 0, 0];
        midpoint(1) = (pos1(1) + pos2(1)) / 2;
        midpoint(2) = (pos1(2) + pos2(2)) / 2;
        midpoint(3) = (pos1(3) + pos2(3)) / 2;
    else
        midpoint = [0, 0];
        midpoint(1) = (pos1(1) + pos2(1)) / 2;
        midpoint(2) = (pos1(2) + pos2(2)) / 2;
    end
end

function [departure_dir, arrival_dir] = calculate_directions(coord1, coord2)
    %CALCULATE_DIRECTIONS Computes the heading angles between two positions 
    % for use in defining Dubins path entry and exit directions.

    % Calculate direction vector between two coordiantes
    direction_vector = coord2 - coord1;
    
    % Determine departure direction
    departure_dir = atan2(direction_vector(2), direction_vector(1));
    % Determine arrival direction (opposite of departure)
    arrival_dir = atan2(direction_vector(2), direction_vector(1));
end

function [departure_dir, arrival_dir] = calculate_directions_3d(coord1, coord2)
    %CALCULATE_DIRECTIONS Computes the heading angles between two positions 
    % for use in defining Dubins path entry and exit directions.

    % Calculate direction vector between two coordiantes
    dir_vec = coord2 - coord1;

    % Calculate x-y plane projection
    horizontal_dist = hypot(dir_vec(1), dir_vec(2));

    % Azimuthal angle: angle in the x-y plane
    azimuth = atan2(dir_vec(2), dir_vec(1));

    % Angle of elevation
    elevation_angle = atan2(dir_vec(3), horizontal_dist);
    
    % Departure direction from coord1 to coord2
    departure_dir = [azimuth, elevation_angle];

    % Reverse vector for arrival direction
    vec_rev = dir_vec;
    horiz_dist_rev = hypot(vec_rev(1), vec_rev(2));
    azimuth_rev = atan2(vec_rev(2), vec_rev(1));
    elevation_rev = atan2(vec_rev(3), horiz_dist_rev);
    
    arrival_dir = [azimuth_rev, elevation_rev];
end

function [path_segment, path_cost] = calculate_dubins_connection(previous_pos, current_pos, next_pos, uav_turning_radius)
    %CALCULATE_DUBINS_CONNECTION Generates a Dubins path segment between two 
    % poses, considering turning radius and heading angles.
    dubConnObj = dubinsConnection;
    dubConnObj.MinTurningRadius = uav_turning_radius;

    [~, prev_arrival_dir] = calculate_directions(previous_pos, current_pos);
    [~, next_arrival_dir] = calculate_directions(current_pos, next_pos);

    start_pose = [current_pos(:, 1:2), prev_arrival_dir];
    goal_pose = [next_pos(:, 1:2), next_arrival_dir];

    [pathSegObj, pathCosts] = connect(dubConnObj, start_pose, goal_pose);
    
    path_segment = pathSegObj;
    path_cost = pathCosts;
end

function [path_segment, path_cost] = calculate_dubins_connection_3d(previous_pos, current_pos, next_pos, uav_turning_radius, uav_airspeed)
    
    dubConnObj = uavDubinsConnection;
    dubConnObj.MaxRollAngle = computeMaxRollAngle(uav_turning_radius, uav_airspeed);

    [~, prev_arrival_dir] = calculate_directions_3d(previous_pos, current_pos);
    [~, next_arrival_dir] = calculate_directions_3d(current_pos, next_pos);
    
    % Index 1 to use azimuth angle (x-y plane angle)
    start_pose = [current_pos, prev_arrival_dir(1)];
    goal_pose = [next_pos, next_arrival_dir(1)];

    [pathSegObj, pathCosts] = connect(dubConnObj, start_pose, goal_pose);
    
    path_segment = pathSegObj;
    path_cost = pathCosts;
end

function [path_segment, path_cost, new_next_pos] = calculate_optimised_dubins_connection(coordinate_waypoints, previous_pos, current_pos, next_pos, uav_turning_radius)
    %CALCULATE_OPTIMISED_DUBINS_CONNECTION Evaluates all 4 candidates in the 
    % target 2x2 imaging grid and chooses the one yielding the lowest Dubins cost.
    dubConnObj = dubinsConnection;
    dubConnObj.MinTurningRadius = uav_turning_radius;
    
    % Find index of next_pos coordinate and retrieve the 4 imaging
    % candidates for the 2x2 grid
    [next_pos_index, ~] = find(ismember(coordinate_waypoints, next_pos, 'rows'));
    possible_next_coordinates = get_image_area_coordinates(coordinate_waypoints, next_pos_index);
    
    % Calculate the arrival direction of the previous path to the current
    % position.
    [~, prev_arrival_dir] = calculate_directions(previous_pos, current_pos);

    path_cost = inf;
    new_next_pos = 0;
    
    % Loop over all imaging 2x2 grid coordinates, and find the path to the
    % coordinate with the shortest cost
    for i = 1:size(possible_next_coordinates, 1)
        current_next_pos = possible_next_coordinates(i, :);
        [~, next_arrival_dir] = calculate_directions(current_pos, current_next_pos);

        start_pose = [current_pos(:, 1:2), prev_arrival_dir];
        goal_pose = [current_next_pos(:, 1:2), next_arrival_dir];

        [pathSegObj, current_path_cost] = connect(dubConnObj, start_pose, goal_pose);

        if current_path_cost < path_cost
            path_cost = current_path_cost;
            path_segment = pathSegObj;
            new_next_pos = current_next_pos;
        end
    end
end

function [path_segment, path_cost, new_next_pos] = calculate_optimised_dubins_connection_3d(coordinate_waypoints, previous_pos, current_pos, next_pos, uav_turning_radius, uav_airspeed)
    
    dubConnObj = uavDubinsConnection;
    dubConnObj.MaxRollAngle = computeMaxRollAngle(uav_turning_radius, uav_airspeed);

    % Find index of next_pos coordinate and retrieve the 4 imaging
    % candidates for the cuboid
    [next_pos_index, ~] = find(ismember(coordinate_waypoints, next_pos, 'rows'));
    possible_next_coordinates = get_image_area_coordinates_3d(coordinate_waypoints, next_pos_index);
    
    % Calculate the arrival direction of the previous path to the current
    % position.
    [~, prev_arrival_dir] = calculate_directions_3d(previous_pos, current_pos);

    path_cost = inf;
    new_next_pos = 0;

    % Loop over all imaging cuboid coordinates, and find the path to the
    % coordinate with the shortest cost
    for i = 1:size(possible_next_coordinates, 1)
        current_next_pos = possible_next_coordinates(i, :);
        [~, next_arrival_dir] = calculate_directions_3d(current_pos, current_next_pos);

        start_pose = [current_pos, prev_arrival_dir(1)];
        goal_pose = [current_next_pos, next_arrival_dir(1)];

        [pathSegObj, current_path_cost] = connect(dubConnObj, start_pose, goal_pose);

        if current_path_cost < path_cost
            path_cost = current_path_cost;
            path_segment = pathSegObj;
            new_next_pos = current_next_pos;
        end
    end
end

function rollAngleRad = computeMaxRollAngle(turnRadius, airspeed)
    % computeMaxRollAngle calculates the maximum roll angle (in degrees)
    % required for a given minimum turning radius and airspeed
    %
    % Inputs:
    %   turnRadius - minimum turning radius in meters
    %   airspeed   - true airspeed in m/s
    %
    % Output:
    %   maxRollAngleDeg - maximum roll angle in degrees

    g = 9.81; % gravitational acceleration in m/s^2

    % Compute roll angle in radians
    rollAngleRad = atan((airspeed^2) / (g * turnRadius));
end

function coordinate_waypoints_3d = setup_elevation_data(coordinate_waypoints_2d)
    min_height = 0;
    max_height = 50;
    num_points = size(coordinate_waypoints_2d, 1);
    % Create linearly spaced elevations from min to max
    gradual_heights = floor(linspace(min_height, max_height, num_points)');

    % Combine 2D coordinates with the gradual elevation
    coordinate_waypoints_3d = [coordinate_waypoints_2d, gradual_heights];
end

function unexplored_coordinate_points = reinsert_coordinates(unexplored_coordinate_points, coordinate_waypoints, index)
    %REINSERT_COORDINATES Reinserts a 2x2 grid's 4 coordinates back into the 
    % unexplored list, useful for heuristic re-planning.
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