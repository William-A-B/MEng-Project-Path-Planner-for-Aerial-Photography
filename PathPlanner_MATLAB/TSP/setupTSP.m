function [coordinate_path, dubins_path_waypoints] = setupTSP(start_pos, goal_pos, polygon_vertices, wind_direction, altitude_limits, uav_turning_radius, num_divisions, plot_results)

    %% =========================================================================
    % REAL COORDINATE TEST
    polygon_vertices = round(polygon_vertices, 6);
    start_pos = round(start_pos, 6);
    goal_pos = round(goal_pos, 6);
    
    polygon_vertices_2d = polygon_vertices(:, 1:2);
    
    % Scale values to UTM
    [utm_polygon_vertices(:, 1), utm_polygon_vertices(:, 2), polygon_utmzone] = deg2utm(polygon_vertices_2d(:, 1), polygon_vertices_2d(:, 2));
    
    [start_pos_utm(:, 1), start_pos_utm(:, 2), start_pos_utmzone] = deg2utm(start_pos(:, 1), start_pos(:, 2));
    start_pos_utm = [start_pos_utm, start_pos(:, 3)];
    [goal_pos_utm(:, 1), goal_pos_utm(:, 2), goal_pos_utmzone] = deg2utm(goal_pos(:, 1), goal_pos(:, 2));
    goal_pos_utm = [goal_pos_utm, goal_pos(:, 3)];

    polygon_vertices_utm = [utm_polygon_vertices, polygon_vertices(:, 3)];
    
    % Define grid resolution (adjust as needed)
    num_divisions_x = num_divisions.x;  % Number of divisions along latitude
    num_divisions_y = num_divisions.y;  % Number of divisions along longitude
    num_divisions_z = num_divisions.z;  % Number of divisions along altitude
    
    lat_min = min(polygon_vertices_utm(:,1));
    lat_max = max(polygon_vertices_utm(:,1));
    lon_min = min(polygon_vertices_utm(:,2));
    lon_max = max(polygon_vertices_utm(:,2));
    alt_min = altitude_limits.min;
    alt_max = altitude_limits.max;
    
    % Generate a grid of waypoints
    lat_values = linspace(lat_min, lat_max, num_divisions_x);
    lon_values = linspace(lon_min, lon_max, num_divisions_y);
    alt_values = linspace(alt_min, alt_max, num_divisions_z);
    [lon_grid, lat_grid, alt_grid] = ndgrid(lon_values, lat_values, alt_values);
    
    % Convert to list of waypoints
    imaging_coordinates = [lat_grid(:), lon_grid(:), alt_grid(:)];

    % Remove any points outside the polygon region
    [in, on] = inpolygon(imaging_coordinates(:, 1), imaging_coordinates(:, 2), polygon_vertices_utm(:,1), polygon_vertices_utm(:,2));
    valid_imaging_coordinates_indices = in | on;
    valid_imaging_coordinates = imaging_coordinates(valid_imaging_coordinates_indices, :);

    % Calculate all real altitudes for each grid point
    valid_imaging_coordinates_deg = utm2deg_wrapper(valid_imaging_coordinates, polygon_utmzone);
    imaging_coordinates_altitudes = getElevation(valid_imaging_coordinates_deg(:, 1), valid_imaging_coordinates_deg(:, 2));
    uav_imaging_coordinate_altitudes = valid_imaging_coordinates(:, 3) + imaging_coordinates_altitudes;
    valid_imaging_coordinates_real_altitudes = [valid_imaging_coordinates(:, 1), valid_imaging_coordinates(:, 2), uav_imaging_coordinate_altitudes];

    if plot_results
        display_initial_coordinate_grid(polygon_vertices_utm, imaging_coordinates, valid_imaging_coordinates);
        display_imaging_coordinate_3d_grid(valid_imaging_coordinates_real_altitudes);
    end
    
    square_size = [range(polygon_vertices_utm(:, 1))/(num_divisions_x), range(polygon_vertices_utm(:, 2))/(num_divisions_y)]; 
    square_centres = valid_imaging_coordinates_real_altitudes;
    square_corners_2d = calculate_square_corner_coordinates(square_centres, square_size);
    square_corners = [square_corners_2d, zeros(size(square_corners_2d, 1), 1)];

    cuboid_corners = calculate_cuboid_corner_coordinates(valid_imaging_coordinates_real_altitudes, square_size, num_divisions_z);
    
    %% Solve TSP
    [coordinate_path, dubins_path_collection] = solveTravellingSalesmanProblem(start_pos_utm, goal_pos_utm, cuboid_corners, wind_direction, uav_turning_radius, num_divisions_z);
    if plot_results
        display_results(start_pos_utm, goal_pos_utm, wind_direction, square_size(:, 1:2), square_centres, cuboid_corners, coordinate_path, dubins_path_collection, false);
        display_results_clean(start_pos_utm, goal_pos_utm, wind_direction, square_size(:, 1:2), square_centres, cuboid_corners, coordinate_path, dubins_path_collection, false);
    end
    
    dubins_path_waypoints_utm = [];

    for i = 1:size(dubins_path_collection, 1)
        % Evaluate pathSegObj to get path states (poses)
        dubins_path_waypoints_connection = interpolate(dubins_path_collection{i}, linspace(0, dubins_path_collection{i}.Length, 20));
        dubins_path_waypoints_utm = [dubins_path_waypoints_utm; dubins_path_waypoints_connection];
    end
    
    % Update the utmzone array to equal the size of the full list of
    % coordinates
    num_coordinate_points = size(coordinate_path, 1);
    utmzone_updated = repmat(polygon_utmzone(1, :), num_coordinate_points, 1);
    [resulting_lat_coords, resulting_lon_coords] = utm2deg(coordinate_path(:, 1), coordinate_path(:, 2), utmzone_updated);
    coordinate_path = [resulting_lat_coords, resulting_lon_coords];

    num_dubins_waypoints = size(dubins_path_waypoints_utm, 1);
    utmzone_dubins_waypoints = repmat(polygon_utmzone(1, :), num_dubins_waypoints, 1);
    [dubins_lat_coords, dubins_lon_coords] = utm2deg(dubins_path_waypoints_utm(:, 1), dubins_path_waypoints_utm(:, 2), utmzone_dubins_waypoints);
    dubins_path_waypoints = [dubins_lat_coords, dubins_lon_coords];
    
end

%% HELPER FUNCTIONS

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

function [cuboid_corners] = calculate_cuboid_corner_coordinates(imaging_coordinates, square_size, num_divisions_z)
    num_cuboids = size(imaging_coordinates, 1) / num_divisions_z;
    square_radius = square_size(1) / 4;
    cuboid_corners = zeros(num_cuboids * num_divisions_z * 4, 3);
    cuboid_index = 1;
    for i = 1:num_cuboids
        cuboid_centre = imaging_coordinates(i, :);
        [cuboid_centre_indexes, ~] = find(ismember(imaging_coordinates(:, 1:2), cuboid_centre(:, 1:2), 'rows'));
        cuboid_altitudes = imaging_coordinates(cuboid_centre_indexes, 3);
        num_cuboid_altitudes = size(cuboid_altitudes, 1);
        corner_set = zeros(num_cuboid_altitudes * 4, 3);
        corner_altitude_index = 1;
        for j = 1:num_cuboid_altitudes
            corners_at_alt = [cuboid_centre(1) - square_radius, cuboid_centre(2) + square_radius, cuboid_altitudes(j);
                          cuboid_centre(1) + square_radius, cuboid_centre(2) + square_radius, cuboid_altitudes(j);
                          cuboid_centre(1) + square_radius, cuboid_centre(2) - square_radius, cuboid_altitudes(j);
                          cuboid_centre(1) - square_radius, cuboid_centre(2) - square_radius, cuboid_altitudes(j)];
            corner_set(corner_altitude_index:corner_altitude_index+3, :) = corners_at_alt;
            corner_altitude_index = corner_altitude_index + 4;
        end
        cuboid_upper_index = (cuboid_index + ((num_cuboid_altitudes * 4) - 1));
        cuboid_corners(cuboid_index:cuboid_upper_index, :) = corner_set;
        cuboid_index = cuboid_upper_index + 1;
    end
end

function [coordinate_list] = utm2deg_wrapper(utm_coordinate_list, utmzone)
    num_coordinate_points = size(utm_coordinate_list, 1);
    utmzone_updated = repmat(utmzone(1, :), num_coordinate_points, 1);
    [resulting_lat_coords, resulting_lon_coords] = utm2deg(utm_coordinate_list(:, 1), utm_coordinate_list(:, 2), utmzone_updated);
    coordinate_list = [resulting_lat_coords, resulting_lon_coords, utm_coordinate_list(:, 3)];
end

function display_results(start_pos, goal_pos, wind_direction, square_size, square_centres, cuboid_corners, coordinate_path, dubins_paths, is_2d)
    % Display results
    figure;
    hold on;
    grid on;
    
    for i = 1:size(dubins_paths, 1)
        % show(paths{i});
        % Evaluate pathSegObj to get path states (poses)
        interpStates = interpolate(dubins_paths{i}, linspace(0, dubins_paths{i}.Length, 50));

        if is_2d
            % Plot only the path line (fast)
            plot(interpStates(:,1), interpStates(:,2), 'b-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        else
            % Plot only the path line (fast)
            plot3(interpStates(:,1), interpStates(:,2), interpStates(:,3), 'b-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
    end
    
    % Plot wind direction
    % Compute wind vector components
    wind_x = sin(wind_direction); 
    wind_y = cos(wind_direction);
    
    % Define arrow starting position (e.g., near the start position)
    arrow_start = start_pos(1, 1:2) - [wind_x, wind_y] * 2; % Shift back slightly for clarity
    
    % Plot the wind direction as an arrow
    % quiver(arrow_start(1), arrow_start(2), wind_x, wind_y, range(polygon_vertices(:, 1))/5, 'k', 'LineWidth', 2, 'MaxHeadSize', 20, 'DisplayName', 'Wind Direction');
    quiver(arrow_start(1), arrow_start(2), wind_x, wind_y, 40, 'k', 'LineWidth', 2, 'MaxHeadSize', 20, 'DisplayName', 'Wind Direction');
    
    if is_2d
        % Plot start and goal positions
        plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Position'); % Start
        plot(goal_pos(1), goal_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal Position'); % Goal
    else
        % Plot start and goal positions
        plot3(start_pos(1), start_pos(2), start_pos(3), 'go', 'MarkerSize', 10, ...
            'MarkerFaceColor', 'g', 'DisplayName', 'Start Position');
        plot3(goal_pos(1), goal_pos(2), goal_pos(3), 'ro', 'MarkerSize', 10, ...
            'MarkerFaceColor', 'r', 'DisplayName', 'Goal Position');
    end
    if is_2d
        % Draw squares at each center position in red
        num_points = size(square_centres, 1);
        for i = 1:num_points
            % Define bottom-left corner of square
            bottom_left = square_centres(i, 1:2) - square_size / 4;
            % Draw red square
            rectangle('Position', [bottom_left, square_size/2], 'EdgeColor', 'c', 'LineWidth', 1);
        end
    else
        % Plot square centers (flat, constant Z)
        num_points = size(square_centres, 1);
        for i = 1:num_points
            center = square_centres(i, 1:3);
            bottom_left = center(1:2) - square_size / 4;
            z = center(3);
            % Plot as square in XY plane at given Z
            plot3([bottom_left(1), bottom_left(1)+square_size(1)/2, bottom_left(1)+square_size(1)/2, bottom_left(1), bottom_left(1)], ...
                  [bottom_left(2), bottom_left(2), bottom_left(2)+square_size(2)/2, bottom_left(2)+square_size(2)/2, bottom_left(2)], ...
                  repmat(z, 1, 5), 'c-', 'LineWidth', 1, 'HandleVisibility', 'off');
        end
    end
    
    if is_2d
        % Plot cuboid centres
        scatter(cuboid_corners(:,1), cuboid_corners(:,2), 20, 'filled', 'b', 'DisplayName', 'Imaging Positions');
    else
        % Plot cuboid corners (imaging positions)
        scatter3(cuboid_corners(:,1), cuboid_corners(:,2), cuboid_corners(:,3), ...
                20, 'filled', 'b', 'DisplayName', 'Imaging Positions');
    end
    
    if is_2d
        plot(coordinate_path(:,1), coordinate_path(:,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');
    else
        % Plot coordinate path in 3D
        plot3(coordinate_path(:,1), coordinate_path(:,2), coordinate_path(:,3), ...
            'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');
    end

    % Add legend to clarify the wind direction
    legend('show');
end

function display_results_clean(start_pos, goal_pos, wind_direction, square_size, square_centres, cuboid_corners, coordinate_path, dubins_paths, is_2d)
    figure;
    hold on;
    grid on;

    % Plot Dubins paths
    for i = 1:size(dubins_paths, 1)
        interpStates = interpolate(dubins_paths{i}, linspace(0, dubins_paths{i}.Length, 50));
        if is_2d
            plot(interpStates(:,1), interpStates(:,2), 'b-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        else
            plot3(interpStates(:,1), interpStates(:,2), interpStates(:,3), 'b-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
    end

    % Wind vector
    wind_x = sin(wind_direction); 
    wind_y = cos(wind_direction);
    arrow_start = start_pos(1, 1:2) - [wind_x, wind_y] * 2;
    quiver(arrow_start(1), arrow_start(2), wind_x, wind_y, 40, 'k', 'LineWidth', 2, 'MaxHeadSize', 20, 'DisplayName', 'Wind Direction');

    % Plot start/goal
    if is_2d
        plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Position');
        plot(goal_pos(1), goal_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal Position');
    else
        plot3(start_pos(1), start_pos(2), start_pos(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Position');
        plot3(goal_pos(1), goal_pos(2), goal_pos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal Position');
    end

    % Determine altitude-to-color mapping
    if ~is_2d
        unique_z = unique(round(coordinate_path(:,3), 3));  % rounded Zs
        cmap = jet(length(unique_z));  % Blue → Green → Yellow → Red
        z_to_color = containers.Map('KeyType', 'double', 'ValueType', 'any');
        for i = 1:length(unique_z)
            z_to_color(unique_z(i)) = cmap(i,:);
        end
    end

    % Proximity threshold
    proximity_thresh = max(square_size) / 2;

    % Plot colored squares
    if is_2d
        for i = 1:size(square_centres, 1)
            center = square_centres(i, 1:2);
            distances = vecnorm(coordinate_path(:, 1:2) - center, 2, 2);
            if any(distances < proximity_thresh)
                bottom_left = center - square_size / 4;
                rectangle('Position', [bottom_left, square_size/2], 'EdgeColor', 'c', 'LineWidth', 1);
            end
        end
    else
        for i = 1:size(square_centres, 1)
            center = square_centres(i, :);
            z = round(center(3), 3);
            z_match = abs(coordinate_path(:,3) - z) < 1e-3;
            xy_dist = vecnorm(coordinate_path(:, 1:2) - center(1:2), 2, 2);
            if any(xy_dist(z_match) < proximity_thresh)
                color = z_to_color(z);
                bottom_left = center(1:2) - square_size(1:2)/4;
                plot3([bottom_left(1), bottom_left(1)+square_size(1)/2, bottom_left(1)+square_size(1)/2, bottom_left(1), bottom_left(1)], ...
                      [bottom_left(2), bottom_left(2), bottom_left(2)+square_size(2)/2, bottom_left(2)+square_size(2)/2, bottom_left(2)], ...
                      repmat(z, 1, 5), 'Color', color, 'LineWidth', 1, 'HandleVisibility', 'off');
            end
        end
    end

    % Plot colored cuboid corners
    if is_2d
        distances = pdist2(coordinate_path(:, 1:2), cuboid_corners(:, 1:2));
        relevant = any(distances < proximity_thresh, 1);
        scatter(cuboid_corners(relevant,1), cuboid_corners(relevant,2), 20, 'filled', 'b', 'DisplayName', 'Imaging Positions');
    else
        for i = 1:size(cuboid_corners, 1)
            corner = cuboid_corners(i, :);
            z = round(corner(3), 3);
            z_match = abs(coordinate_path(:,3) - z) < 1e-3;
            xy_dist = vecnorm(coordinate_path(:,1:2) - corner(1:2), 2, 2);
            if any(xy_dist(z_match) < proximity_thresh)
                color = z_to_color(z);
                scatter3(corner(1), corner(2), corner(3), 40, 'filled', 'MarkerEdgeColor', color, 'MarkerFaceColor', color, 'HandleVisibility', 'off');
            end
        end
    end

    % Plot original path
    if is_2d
        plot(coordinate_path(:,1), coordinate_path(:,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');
    else
        plot3(coordinate_path(:,1), coordinate_path(:,2), coordinate_path(:,3), ...
              'r--o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Original Path');
    end

    if ~is_2d
        colormap(jet(length(unique_z)));
        caxis([min(unique_z), max(unique_z)]);
        colorbar('Ticks', linspace(min(unique_z), max(unique_z), 5), ...
                 'TickLabels', round(linspace(min(unique_z), max(unique_z), 5)));
    end

    legend('show');
end




function display_initial_coordinate_grid(polygon_vertices_utm, grid_waypoints, valid_grid_waypoints)
    % Plot the polygon
    figure;
    hold on;
    % Plot polygon
    plot(polygon_vertices_utm(:,1), polygon_vertices_utm(:,2), 'b-o', 'LineWidth', 2);
    xlabel('Longitude');
    ylabel('Latitude');
    title('Defined Surveillance Area');
    grid on;
    axis equal;

    % Plot the grid points
    scatter(grid_waypoints(:,1), grid_waypoints(:,2), 'r*');
    scatter(valid_grid_waypoints(:,1), valid_grid_waypoints(:,2), 'g*');
    legend('Polygon Boundary', 'Grid Points');
end

function display_imaging_coordinate_3d_grid(imaging_coordinates)
    % Extract UTM X, Y, and real-world altitude (Z)
    x = imaging_coordinates(:,1);
    y = imaging_coordinates(:,2);
    z = imaging_coordinates(:,3);
    
    % Create an interpolant function from the scattered data
    F = scatteredInterpolant(x, y, z, 'natural', 'none');

    % Define regular grid resolution (adjust as needed)
    grid_res = 100;  % number of points along each axis
    
    % Create mesh grid over the range of your data
    x_lin = linspace(min(x), max(x), grid_res);
    y_lin = linspace(min(y), max(y), grid_res);
    [Xq, Yq] = meshgrid(x_lin, y_lin);
    
    % Interpolate Z values (altitude) over the regular grid
    Zq = F(Xq, Yq);
    
    figure;
    mesh(Xq, Yq, Zq);  % Or use surf(Xq, Yq, Zq) for filled surface
    xlabel('X (UTM)');
    ylabel('Y (UTM)');
    zlabel('Altitude (m)');
    title('Interpolated UAV Altitude Surface');
    colormap jet;
    colorbar;
    shading interp;    % Smooth color transitions
    view(3);
    grid on;

    hold on;
    scatter3(x, y, z, 10, 'k', 'filled');
end

function display_imaging_coordinate_3d_grid_simple(imaging_coordinates)
    figure;
    scatter3( ...
        imaging_coordinates(:,1), ...  % X (UTM)
        imaging_coordinates(:,2), ...  % Y (UTM)
        imaging_coordinates(:,3), ... % Z (real altitude)
        36, ...                               % Marker size
        imaging_coordinates(:,3), ... % Color by altitude
        'filled' ...
    );
    xlabel('X (UTM)');
    ylabel('Y (UTM)');
    zlabel('Altitude (m)');
    title('3D UAV Waypoints with Altitude Gradient');
    colormap jet;        % You can use parula, viridis, turbo, etc.
    colorbar;            % Show color scale for altitude
    view(3);             % 3D view
    grid on;

end