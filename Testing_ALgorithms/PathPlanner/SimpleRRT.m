% Simple 2D RRT Algorithm in MATLAB
clc; clear; close all;

% Parameters
start = [0, 0];
goal = [99, 99];
step_size = 0.5;
goal_threshold = 0.5;
num_nodes = 10000;
bounds = [0, 100; 0, 100];

% Obstacles (define as rectangles [x, y, width, height])
obstacles = [30, 30, 20, 40; 60, 50, 20, 20];

% Initialize Tree
nodes = start;
parent = 1;

% Plot environment
figure; hold on; axis equal;
xlim(bounds(1, :)); ylim(bounds(2, :));
rectangle('Position', [goal 0.5 0.5], 'FaceColor', 'g');
for i = 1:size(obstacles, 1)
    rectangle('Position', obstacles(i, :), 'FaceColor', 'r');
end

% Main RRT Loop
for i = 1:num_nodes
    % Random Sampling
    if rand < 0.1
        sample = goal; % Bias towards goal
    else
        sample = [rand * bounds(1, 2), rand * bounds(2, 2)];
    end
    
    % Find Nearest Node
    distances = vecnorm(nodes - sample, 2, 2);
    [~, nearest_idx] = min(distances);
    nearest_node = nodes(nearest_idx, :);

    % Steer Towards Sample
    direction = (sample - nearest_node) / norm(sample - nearest_node);
    new_node = nearest_node + step_size * direction;

    % Collision Check
    if ~isCollision(nearest_node, new_node, obstacles)
        nodes = [nodes; new_node];
        parent = [parent; nearest_idx];
        plot([nearest_node(1), new_node(1)], [nearest_node(2), new_node(2)], 'b');
        drawnow;

        % Goal Check
        if norm(new_node - goal) < goal_threshold
            disp('Goal Reached!');
            break;
        end
    end
end

% Path Extraction
path = [goal];
current_idx = size(nodes, 1);
while current_idx ~= 1
    path = [nodes(current_idx, :); path];
    current_idx = parent(current_idx);
end
plot(path(:, 1), path(:, 2), 'k', 'LineWidth', 2);

function collision = isCollision(p1, p2, obstacles)
    collision = false;
    for i = 1:size(obstacles, 1)
        obs = obstacles(i, :);
        if lineIntersectsRect(p1, p2, obs)
            collision = true;
            return;
        end
    end
end

function intersects = lineIntersectsRect(p1, p2, rect)
    % Check if a line segment intersects a rectangle using simple checks
    xMin = rect(1); xMax = rect(1) + rect(3);
    yMin = rect(2); yMax = rect(2) + rect(4);
    intersects = false;
    % Bounding box check
    if max(p1(1), p2(1)) < xMin || min(p1(1), p2(1)) > xMax || ...
       max(p1(2), p2(2)) < yMin || min(p1(2), p2(2)) > yMax
        return;
    end
    intersects = true;
end
