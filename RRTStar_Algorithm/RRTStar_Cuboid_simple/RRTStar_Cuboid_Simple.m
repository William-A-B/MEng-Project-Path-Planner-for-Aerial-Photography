x_range = [0, 100]; % meters
y_range = [0, 100]; 
z_range = [10, 20]; % Altitude range
cuboid_size = [10, 10, 5]; % Each cuboid's dimensions

[x, y, z] = ndgrid(x_range(1):cuboid_size(1):x_range(2), ...
                    y_range(1):cuboid_size(2):y_range(2), ...
                    z_range(1):cuboid_size(3):z_range(2));
cuboid_centers = [x(:), y(:), z(:)];

start_pos = [0, 0, 15];
goal_pos = [100, 100, 15];

nodes = Node(start_pos, [], 0);
max_iter = 1000;
search_radius = 15; % Meters
goal_tolerance = 5; % Meters

directions = [0, 90; 90, 180; 45, 135];


energies = zeros(size(directions, 1), 1);
paths = cell(size(directions, 1), 1);

for i = 1:size(directions, 1)
    % Adjust search to follow a lawnmower path in given direction
    paths{i} = RRTStar(start_pos, goal_pos, cuboid_centers, directions(i, :));
    energies(i) = calculatePathEnergy(paths{i});
end

% Select the best path
[~, best_path_idx] = min(energies);
best_path = paths{best_path_idx};


figure;
hold on;
scatter3(cuboid_centers(:,1), cuboid_centers(:,2), cuboid_centers(:,3), 'k.');
plot3(start_pos(1), start_pos(2), start_pos(3), 'go', 'MarkerSize', 10);
plot3(goal_pos(1), goal_pos(2), goal_pos(3), 'ro', 'MarkerSize', 10);

% Plot the best path
for i = 1:length(best_path) - 1
    plot3([best_path{i}.position(1), best_path{i+1}.position(1)], ...
          [best_path{i}.position(2), best_path{i+1}.position(2)], ...
          [best_path{i}.position(3), best_path{i+1}.position(3)], 'b-', 'LineWidth', 2);
end
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Optimal UAV Path using RRT*');



function sample_pos = randomSample(x_range, y_range, z_range)
    sample_pos = [rand*(x_range(2) - x_range(1)) + x_range(1), ...
                  rand*(y_range(2) - y_range(1)) + y_range(1), ...
                  rand*(z_range(2) - z_range(1)) + z_range(1)];
end

function nearest_node = findNearest(nodes, sample_pos)
    distances = arrayfun(@(n) norm(n.position - sample_pos), nodes);
    [~, idx] = min(distances);
    nearest_node = nodes(idx);
end

function collision = checkCollision(pos1, pos2, obstacles)
    % Implement your own collision check here
    collision = false; 
end

function new_node = extendTree(nearest_node, sample_pos, step_size)
    direction = (sample_pos - nearest_node.position);
    direction = direction / norm(direction);
    new_pos = nearest_node.position + step_size * direction;
    cost = nearest_node.cost + norm(direction) * step_size;
    new_node = Node(new_pos, nearest_node, cost);
end

function energy = energyCost(node1, node2)
    k1 = 1; % Distance weight
    k2 = 5; % Turn weight
    k3 = 2; % Altitude weight
    
    dist = norm(node2.position - node1.position);
    turn_angle = acos(dot(node1.position - node1.parent.position, ...
                           node2.position - node1.position) ...
                           / (norm(node1.position - node1.parent.position) ...
                              * norm(node2.position - node1.position)));
    altitude_change = abs(node2.position(3) - node1.position(3));
    
    energy = k1 * dist + k2 * turn_angle + k3 * altitude_change;
end



