% Setup environment and algorithm constraints
% Size of environment
x_max = 100;
y_max = 100;

x_range = [0, x_max]; % meters
y_range = [0, y_max]; 
z_range = [10, 30]; % Altitude range
cuboid_size = [10, 10, 20]; % Each cuboid's dimensions

[x, y, z] = ndgrid(x_range(1):cuboid_size(1):x_range(2), ...
                    y_range(1):cuboid_size(2):y_range(2), ...
                    z_range(1):cuboid_size(3):z_range(2));
cuboid_centers = [x(:), y(:), z(:)];

start_pos = [0, 0, 20];
goal_pos = [100, 100, 20];

% Plot cuboid centers
figure;
scatter3(cuboid_centers(:,1), cuboid_centers(:,2), cuboid_centers(:,3), 'filled');
hold on;

% Plot start and goal positions
plot3(start_pos(1), start_pos(2), start_pos(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
plot3(goal_pos(1), goal_pos(2), goal_pos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Goal


% Connect cuboid centers to form a grid
num_points = size(cuboid_centers, 1);
for i = 1:num_points
    for j = i+1:num_points
        % Check if the points are adjacent in x, y, or z direction
        diff_vector = abs(cuboid_centers(i,:) - cuboid_centers(j,:));
        if sum(diff_vector == [cuboid_size(1), 0, 0]) == 3 || ...
           sum(diff_vector == [0, cuboid_size(2), 0]) == 3 || ...
           sum(diff_vector == [0, 0, cuboid_size(3)]) == 3
            plot3([cuboid_centers(i,1), cuboid_centers(j,1)], ...
                  [cuboid_centers(i,2), cuboid_centers(j,2)], ...
                  [cuboid_centers(i,3), cuboid_centers(j,3)], 'k');
        end
    end
end

% Labels and Grid
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Cuboid Centers and Start/Goal Positions');
grid on;
axis equal;
hold off;