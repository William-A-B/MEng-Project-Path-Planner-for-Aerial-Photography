function path = RRTStar(start_pos, goal_pos, cuboid_centers, direction)
    % Parameters
    max_iter = 1000;
    search_radius = 15; % Meters
    step_size = 10;
    goal_tolerance = 5;
    nodes = Node(start_pos, [], 0);
    cuboid_visited = false(size(cuboid_centers, 1), 1);

    % Main RRT* Loop
    for i = 1:max_iter
        % Random Sampling
        if rand < 0.1
            sample_pos = goal_pos; % Goal Bias
        else
            sample_pos = randomSample(min(start_pos, goal_pos), max(start_pos, goal_pos));
        end

        % Find Nearest Node
        nearest_node = findNearest(nodes, sample_pos);

        % Extend the Tree
        new_node = extendTree(nearest_node, sample_pos, step_size);
        if checkCollision(nearest_node.position, new_node.position)
            continue;
        end
        
        % Rewire Step: Check Nearby Nodes and Update
        neighbors = findNeighbors(nodes, new_node, search_radius);
        min_cost_node = nearest_node;
        min_cost = nearest_node.cost + norm(new_node.position - nearest_node.position);

        for neighbor = neighbors
            temp_cost = neighbor.cost + norm(new_node.position - neighbor.position);
            if temp_cost < min_cost
                min_cost = temp_cost;
                min_cost_node = neighbor;
            end
        end
        new_node.parent = min_cost_node;
        new_node.cost = min_cost;

        % Add to Nodes
        nodes = [nodes, new_node];

        % Rewiring Step
        for neighbor = neighbors
            if neighbor == new_node.parent
                continue;
            end
            temp_cost = new_node.cost + norm(neighbor.position - new_node.position);
            if temp_cost < neighbor.cost
                neighbor.parent = new_node;
                neighbor.cost = temp_cost;
            end
        end

        % Track Cuboid Coverage
        cuboid_idx = findCuboid(new_node.position, cuboid_centers);
        if cuboid_idx > 0
            cuboid_visited(cuboid_idx) = true;
        end

        % Check Goal Reached
        if norm(new_node.position - goal_pos) < goal_tolerance && all(cuboid_visited)
            disp('Goal Reached!');
            break;
        end
    end

    % Extract Final Path
    path = extractPath(nodes, goal_pos);
end


function path = extractPath(nodes, goal_pos)
    % Find nearest to goal
    distances = arrayfun(@(n) norm(n.position - goal_pos), nodes);
    [~, goal_idx] = min(distances);
    goal_node = nodes(goal_idx);

    % Backtrack to extract path
    path = {};
    while ~isempty(goal_node)
        path = [{goal_node}, path];
        goal_node = goal_node.parent;
    end
end

function cuboid_idx = findCuboid(position, cuboid_centers)
    dists = vecnorm(cuboid_centers - position, 2, 2);
    [min_dist, cuboid_idx] = min(dists);
    if min_dist > 10
        cuboid_idx = -1; % Not in any cuboid
    end
end

function new_node = extendTree(nearest_node, sample_pos, step_size)
    direction = (sample_pos - nearest_node.position);
    direction = direction / norm(direction);
    new_pos = nearest_node.position + step_size * direction;
    cost = nearest_node.cost + norm(new_pos - nearest_node.position);
    new_node = Node(new_pos, nearest_node, cost);
end

function nearest_node = findNearest(nodes, sample_pos)
    distances = arrayfun(@(n) norm(n.position - sample_pos), nodes);
    [~, idx] = min(distances);
    nearest_node = nodes(idx);
end

function sample_pos = randomSample(min_pos, max_pos)
    sample_pos = min_pos + rand(1, 3) .* (max_pos - min_pos);
end

function collision = checkCollision(pos1, pos2)
    % Define your area boundaries
    x_range = [0, 100];
    y_range = [0, 100];
    z_range = [10, 20];
    
    % Check if pos2 is outside the boundaries
    collision = (pos2(1) < x_range(1) || pos2(1) > x_range(2) || ...
                 pos2(2) < y_range(1) || pos2(2) > y_range(2) || ...
                 pos2(3) < z_range(1) || pos2(3) > z_range(2));
end

function neighbors = findNeighbors(nodes, new_node, search_radius)
    % Find all nodes within the given search radius
    distances = arrayfun(@(n) norm(n.position - new_node.position), nodes);
    neighbors = nodes(distances <= search_radius);
end
