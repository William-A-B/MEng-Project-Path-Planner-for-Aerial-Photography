% Define the map (no obstacles here, or use binaryOccupancyMap if needed)
map = binaryOccupancyMap(500, 500, 1);  % 500x500 grid with 1 m resolution

% Create the planner with Dubins motion model
planner = pathPlannerRRT(map);
planner.MinTurningRadius = 20; % meters
planner.ConnectionMethod = "Dubins";

% Define poses [x y theta]
startPose = [50 50 0];       % Start
midPose   = [250 250 pi/2];  % Waypoint
goalPose  = [450 50 0];      % Goal

% Plan start -> mid
path1 = plan(planner, startPose, midPose);

% Plan mid -> goal
path2 = plan(planner, midPose, goalPose);

% Extract and combine poses
combinedStates = [path1.States; path2.States];

% Plotting
figure; 
show(map); hold on;
plot(combinedStates(:,1), combinedStates(:,2), 'r-', 'LineWidth', 2);
plot(startPose(1), startPose(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(midPose(1), midPose(2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Waypoint');
plot(goalPose(1), goalPose(2), 'mo', 'MarkerSize', 10, 'DisplayName', 'Goal');
legend;