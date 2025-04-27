polygon_vertices = [
    0, 0, 0;
    0, 100, 0;
    100, 100, 0;
    100, 0, 0;
];

start_pos = [0, 0, 0];
goal_pos = [100, 100, 0];


% Define grid resolution (adjust as needed)
num_divisions_x = 10;  % Number of divisions along latitude
num_divisions_y = 10;  % Number of divisions along longitude
num_divisions_z = 3;  % Number of divisions along altitude

lat_min = min(polygon_vertices(:,1));
lat_max = max(polygon_vertices(:,1));
lon_min = min(polygon_vertices(:,2));
lon_max = max(polygon_vertices(:,2));
alt_min = 0;
alt_max = 10;

% Generate a grid of waypoints
lat_values = linspace(lat_min, lat_max, num_divisions_x);
lon_values = linspace(lon_min, lon_max, num_divisions_y);
alt_values = linspace(alt_min, alt_max, num_divisions_z);
[lon_grid, lat_grid, alt_grid] = ndgrid(lon_values, lat_values, alt_values);

% Convert to list of waypoints
imaging_coordinates = [lat_grid(:), lon_grid(:), alt_grid(:)];

% Remove any points outside the polygon region
[in, on] = inpolygon(imaging_coordinates(:, 1), imaging_coordinates(:, 2), polygon_vertices(:,1), polygon_vertices(:,2));
valid_imaging_coordinates_indices = in | on;
valid_imaging_coordinates = imaging_coordinates(valid_imaging_coordinates_indices, :);

% Calculate all altitudes for each grid point
random_alt_heights = (alt_min + (alt_max - alt_min) * rand(1, size(valid_imaging_coordinates, 1)))';
imaging_coordinate_altitudes = valid_imaging_coordinates(:, 3) + random_alt_heights;
valid_imaging_coordinates = [valid_imaging_coordinates(:, 1), valid_imaging_coordinates(:, 2), imaging_coordinate_altitudes];


if plot_results
    % display_initial_coordinate_grid(polygon_vertices, imaging_coordinates, valid_imaging_coordinates);
    % display_imaging_coordinate_3d_grid(valid_imaging_coordinates);
end

square_size = [range(polygon_vertices(:, 1))/(num_divisions_x), range(polygon_vertices(:, 2))/(num_divisions_y)]; 
square_centres = valid_imaging_coordinates;
square_corners_2d = calculate_square_corner_coordinates(square_centres, square_size);
square_corners = [square_corners_2d, zeros(size(square_corners_2d, 1), 1)];

cuboid_corners = calculate_cuboid_corner_coordinates(valid_imaging_coordinates, square_size, num_divisions_z);


current_pos = [10, 10, 10];
target_pos = [20, 20, 4];
wind_direction = 0;

uav_cost = calculate_flight_path_cost(cuboid_corners, current_pos, target_pos, wind_direction);
disp(uav_cost)


function uav_cost = calculate_flight_path_cost(coordinate_points, current_pos, target_pos, wind_direction)
    max_climb_angle = 0.5236;
    max_climb_rate = 10;
    global num_div_z;

    base_cost_per_metre = 1;
    descent_bonus_factor = 0.5;
    climb_penalty_factor = 2.0;

    % Calculate the 3D distance between points
    dx = target_pos(1) - current_pos(1);
    dy = target_pos(2) - current_pos(2);
    dz = target_pos(3) - current_pos(3);

    % Calculate slope angle from current position to target position
    slope_angle = atan2(dz, sqrt(dx.^2 + dy.^2));
    if slope_angle > max_climb_angle
        uav_cost = inf;
        return
    end

    pos_distance = sqrt(dx.^2 + dy.^2 + dz.^2);
    
    % Calculate base path cost
    base_cost = pos_distance * base_cost_per_metre;

    % Calculate climb/descent cost
    if slope_angle > 0
        climb_cost = slope_angle * climb_penalty_factor;
    elseif slope_angle < 0
        climb_cost = slope_angle * descent_bonus_factor;
    else
        climb_cost = 0;
    end

    % Calculate total cost
    uav_cost = base_cost + climb_cost;
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