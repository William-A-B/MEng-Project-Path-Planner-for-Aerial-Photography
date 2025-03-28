function waypoints = uav_path(grid, start_pos, end_pos)
    
    area_width = width(grid)
    area_height = height(grid)
    swarth_width = 1;


    waypoints = lawnmowerPath(area_width, area_height, swarth_width, start_pos(1), start_pos(2))
    
end


% Setup "simulated" coordinates
%  A --- B 
%  |     |
%  |     |
%  C --- D

A = [0, 0]
B = [10, 0]
C = [0, 10]
D = [10, 10]

% Form a grid to map with
grid = zeros(D(2), D(1));

start_pos = [1, 1];   % Start at (1,1)
end_pos = [D(1), D(2)];   % End at (10,10)
waypoints = uav_path(grid, start_pos, end_pos);