polygon_vertices = [
    53.950974807525206, -1.0329672619323844;
    53.946024591620926, -1.033224753997814;
    53.945898302917456, -1.0243412777404899;
    53.95089904334211, -1.0251566692810172;
    53.950974807525206, -1.0329672619323844 % Close the loop
];

[waypoint_path, square_corners] = PathPlanner2D(polygon_vertices);

% Extract x, y coordinates from all stored waypoints
lat = waypoint_path(:,1);
long = waypoint_path(:,2);

figure;
hold on;
scatter(square_corners(:,1), square_corners(:,2), 40, 'r*');

hold on;
% Plot the full path with all waypoints
plot(lat, long, 'bo-'); % Blue circles for waypoints, line connecting them
hold on;
grid on;
xlabel('Latitude');
ylabel('Longitude');
title('Calculated Waypoint Path');
legend('Imaging Points', 'Waypoint Path');
