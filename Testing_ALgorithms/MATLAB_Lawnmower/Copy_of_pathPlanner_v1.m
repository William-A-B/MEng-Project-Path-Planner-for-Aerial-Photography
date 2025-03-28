function waypoints = uav_dijkstra_path(grid_size, start_pos, end_pos)
    % UAV Path Planning using Dijkstra's Algorithm for Aerial Photography
    % Optimized with sparse matrices and binary heap priority queue
    
    rows = grid_size(1);
    cols = grid_size(2);
    num_nodes = rows * cols;
    
    % Create a sparse adjacency matrix for efficiency
    adj_matrix = sparse(num_nodes, num_nodes);
    
    for r = 1:rows
        for c = 1:cols
            node = (r - 1) * cols + c;
            if c < cols  % Right neighbor
                adj_matrix(node, node + 1) = 1;
                adj_matrix(node + 1, node) = 1;
            end
            if r < rows  % Downward neighbor
                adj_matrix(node, node + cols) = 1;
                adj_matrix(node + cols, node) = 1;
            end
        end
    end
    
    % Convert start/end positions to indices
    start_idx = (start_pos(2) - 1) * cols + start_pos(1);
    end_idx = (end_pos(2) - 1) * cols + end_pos(1);
    
    % Apply Dijkstra's Algorithm with binary heap
    [distances, prev_nodes] = dijkstra_heap(adj_matrix, start_idx);
    
    % Check if end node is reachable
    if isnan(prev_nodes(end_idx))
        error('No path found between start and end positions.');
    end
    
    % Extract the shortest path
    path = reconstruct_path(prev_nodes, start_idx, end_idx);
    
    % Convert node indices back to (x, y) coordinates
    waypoints = arrayfun(@(idx) [(mod(idx-1, cols) + 1), ceil(idx / cols)], path, 'UniformOutput', false);
    waypoints = cell2mat(waypoints);
    
    % Plot the grid and path
    figure;
    hold on;
    gplot(adj_matrix, [mod((1:num_nodes)-1, cols)+1; ceil((1:num_nodes)/cols)]', 'k.');
    plot(waypoints(:,1), waypoints(:,2), 'r-', 'LineWidth', 2);
    title('Optimized UAV Path for Aerial Photography');
    xlabel('X-axis'); ylabel('Y-axis');
    grid on;
end

function [distances, prev_nodes] = dijkstra_heap(graph, start_node)
    num_nodes = size(graph, 1);
    distances = inf(1, num_nodes);
    prev_nodes = nan(1, num_nodes);
    distances(start_node) = 0;
    
    % Min-Heap implementation
    heap = [start_node, 0]; % [node, distance]
    
    while ~isempty(heap)
        % Extract node with smallest distance
        [~, idx] = min(heap(:,2));
        current = heap(idx, 1);
        heap(idx, :) = []; % Remove the node from heap
        
        neighbors = find(graph(current, :) > 0);
        for neighbor = neighbors
            alt = distances(current) + graph(current, neighbor);
            if alt < distances(neighbor)
                distances(neighbor) = alt;
                prev_nodes(neighbor) = current;
                heap = [heap; neighbor, alt]; % Insert into heap
            end
        end
    end
end

function path = reconstruct_path(prev_nodes, start_node, end_node)
    path = end_node;
    while path(1) ~= start_node
        if isnan(prev_nodes(path(1)))
            error('Path reconstruction failed. No valid path.');
        end
        path = [prev_nodes(path(1)), path];
    end
end

% Example usage:
grid_size = [10, 40]; % 10x10 grid
start_pos = [1, 1];   % Start at (1,1)
end_pos = [10, 10];   % End at (10,10)
waypoints = uav_dijkstra_path(grid_size, start_pos, end_pos);