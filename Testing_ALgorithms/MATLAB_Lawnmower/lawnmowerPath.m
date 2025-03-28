function waypoints = lawnmowerPath(areaWidth, areaHeight, swathWidth, startX, startY)
    % Compute number of passes needed
    numPasses = ceil(areaWidth / swathWidth);
    
    % Initialize waypoint storage
    waypoints = zeros(numPasses * 2, 2);
    
    % Generate waypoints
    for i = 1:numPasses
        x = startX + (i-1) * swathWidth; % Current pass x-coordinate
        if mod(i, 2) == 1
            waypoints(2*i-1, :) = [x, startY];          % Start of pass
            waypoints(2*i, :) = [x, startY + areaHeight]; % End of pass
        else
            waypoints(2*i-1, :) = [x, startY + areaHeight]; % Start of pass
            waypoints(2*i, :) = [x, startY];          % End of pass
        end
    end
    
    % Plot the path
    figure;
    plot(waypoints(:,1), waypoints(:,2), 'b-o', 'LineWidth', 2);
    xlabel('Width (m)');
    ylabel('Height (m)');
    title('Lawnmower UAV Path');
    grid on;
    axis equal;
    
end