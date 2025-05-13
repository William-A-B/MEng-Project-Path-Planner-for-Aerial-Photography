% RRT* algorithm in 2D with collision avoidance.
% 
% Author: Sai Vemprala
% 
% nodes:    Contains list of all explored nodes. Each node contains its
%           coordinates, cost to reach and its parent.
% 
% Brief description of algorithm: 
% 1. Pick a random node q_rand.
% 2. Find the closest node q_near from explored nodes to branch out from, towards
%    q_rand.
% 3. Steer from q_near towards q_rand: interpolate if node is too far away, reach
%    q_new. Check that obstacle is not hit.
% 4. Update cost of reaching q_new from q_near, treat it as Cmin. For now,
%    q_near acts as the parent node of q_new.
% 5. From the list of 'visited' nodes, check for nearest neighbors with a 
%    given radius, insert in a list q_nearest.
% 6. In all members of q_nearest, check if q_new can be reached from a
%    different parent node with cost lower than Cmin, and without colliding
%    with the obstacle. Select the node that results in the least cost and 
%    update the parent of q_new.
% 7. Add q_new to node list.
% 8. Continue until maximum number of nodes is reached or goal is hit.

% Clears all variables and closes any figures
clearvars
close all

% Setup environment and algorithm constraints
% Size of environment
x_max = 1000;
y_max = 1000;
% Obstacle
obstacle1 = [100, 0, 20, 900];
obstacle2 = [300, 100, 20, 900];
obstacle3 = [500, 0, 20, 900];
obstacle4 = [700, 100, 20, 900];
obstacle_list = [obstacle1; obstacle2; obstacle3; obstacle4];
% Step size - how far tree can expand per iteration
EPS = 40;
% Max number of nodes the algorithm will generate
numNodes = 2000;        

% Start and goal positions
q_start.coord = [0 0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [999 999];
q_goal.cost = 0;

% Imaging pass spacing
pass_spacing = x_max / 10;

q_goal_temp.coord = [(pass_spacing / 2), (y_max - 1)];
q_goal_temp.cost = 0;

% nodes stores all explored nodes
nodes(1) = q_start;

% Probability to bias towards the goal destination
goal_bias_prob = 0.5;

temp_goal_direction = -1;

% Create figure, draw obstacle
figure(1)
axis([0 x_max 0 y_max])
rectangle('Position',obstacle1,'FaceColor',[.9 .9 .1])
rectangle('Position',obstacle2,'FaceColor',[.9 .9 .1])
rectangle('Position',obstacle3,'FaceColor',[.9 .9 .1])
rectangle('Position',obstacle4,'FaceColor',[.9 .9 .1])
hold on

% Main loop - growing the tree
for i = 1:1:numNodes
    
    % Bias towards the goal
    if rand < goal_bias_prob
        q_rand = q_goal_temp.coord;
    else
        % Generates a random point in the 2D space
        q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    end
    
    % Plot point on figure
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410], MarkerSize=8)
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end

    % Update temporary goal if reached.
    for k = 1:1:length(nodes)
        if nodes(k).coord == q_goal_temp.coord
            q_goal_temp.coord(1) = q_goal_temp.coord(1) + pass_spacing;
            if temp_goal_direction == 1
                q_goal_temp.coord(2) = (y_max - 1);
            else
                q_goal_temp.coord(2) = 1;
            end
            temp_goal_direction = -temp_goal_direction;
        end
    end
    q_goal_temp
    % Pick the closest node to q_rand from existing list to branch out from
    % By calculating the Euclidean distance for all nodes
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    % Move towards q_rand from q_near (limited to EPS num units)
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);

    % Checks if the path between q_near and q_new collides with the
    % obstacle
    if noCollision(q_rand, q_near.coord, obstacle_list)
        % No collision, so draws a line from q_near to q_new
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        % Calculate cost to travel to q_new plus the cost of reaching
        % q_near
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
        % Within a radius of r, find all existing nodes
        % Check if any provide a lower cost path to q_new
        q_nearest = [];
        r = 60;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if noCollision(nodes(j).coord, q_new.coord, obstacle_list) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        % If a better parent is found, updates the connection and draws a
        % green line
        for k = 1:1:length(q_nearest)
            if noCollision(q_nearest(k).coord, q_new.coord, obstacle_list) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g', 'LineWidth', 1);                
                hold on
            end
        end
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Append q_new to explored nodes
        nodes = [nodes q_new];
    end
end

% Finding the optimal path
% Finds the closest node to the goal
D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
% Trace path back to start and draws optimal path.
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodes(start);
end


