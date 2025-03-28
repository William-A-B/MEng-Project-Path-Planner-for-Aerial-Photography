function display2DGrid(start_pos, goal_pos, square_centres, square_size, square_corners, draw_details)
    
    figure;
    hold on;
    % Plot start and goal positions
    plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
    plot(goal_pos(1), goal_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Goal

    if draw_details == true
        % Plot cuboid centers
        scatter(square_centres(:,1), square_centres(:,2), 'filled', 'b');
    
        % Connect cuboid centers to form a grid
        num_points = size(square_centres, 1);
        for i = 1:num_points
            for j = i+1:num_points
                % Check if the points are adjacent in x or y direction
                diff_vector = abs(square_centres(i,:) - square_centres(j,:));
                if sum(diff_vector == [square_size(1), 0]) == 2 || ...
                   sum(diff_vector == [0, square_size(2)]) == 2
                    plot([square_centres(i,1), square_centres(j,1)], [square_centres(i,2), square_centres(j,2)], 'k');
                end
            end
        end
    end

    % Draw squares at each center position in red
    num_points = size(square_centres, 1);
    for i = 1:num_points
        % Define bottom-left corner of square
        bottom_left = square_centres(i,:) - square_size / 4;

        % Draw red square
        rectangle('Position', [bottom_left, square_size/2], 'EdgeColor', 'r', 'LineWidth', 1);
    end

    % Plot cuboid centers
    scatter(square_corners(:,1), square_corners(:,2), 20, 'filled', 'b');
    % 
    % % Connect square centers to form a grid only if a square_centre exists inside
    % num_points = size(square_corners, 1);
    % index = 1;
    % for i = 1:num_points
    %     for j = i+1:num_points
    %         % Check if the points are adjacent in x or y direction
    %         diff_vector = abs(square_corners(i,:) - square_corners(j,:));
    %         if sum(diff_vector == [square_size(1), 0]) == 2 || ...
    %            sum(diff_vector == [0, square_size(2)]) == 2
    % 
    %              % Loop through square_centres to check if any are inside the square area
    %             for k = 1:size(square_centres, 1)  % Loop through all square_centres
    %                 [in, on] = inpolygon(square_centres(k,1), square_centres(k,2), ...
    %                                      square_corners(index:index+3, 1), square_corners(index:index+3, 2));
    % 
    %                 % If any of the square_centres is inside the polygon, draw the line
    %                 if in
    %                     plot([square_corners(i,1), square_corners(j,1)], ...
    %                          [square_corners(i,2), square_corners(j,2)], 'k');
    %                     break; % If one center is inside, no need to check further centers for this pair
    %                 end
    %             end
    %         end
    %     end
    %     if ~mod(i, 4) 
    %         index = index + 4;
    %     end
    % end
    
    % Labels and Grid
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Square Centers and Start/Goal Positions');
    grid on;
    axis equal;
    hold off;
end
