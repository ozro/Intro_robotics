% Inputs:
% map         - configuration space with obstacles as 1 and free space as 0
% dx          - number of inches per element in discretized map
% start, goal - [x, y] of starting and ending positions, in inches
% visualize   - flag for path finding process visualization
function [waypoints, waypoints_inch] = path_gen(map, dx, start, goal, visualize)
% Change map to uint8
map = uint8(map);

% Change to map coorinates
start = round(start/dx);
goal = round(goal/dx);

% Calculate path using A*
fprintf('Running A* algorithm...');
tic
[path,f] = a_star(map, start, goal, visualize);
fprintf(' Complete!\n\tElapsed time is %f s\n\n', toc);

% Refine the path to key waypoints
fprintf('Running path cleaner...');
tic
waypoints = clean_path(path);
fprintf(' Complete!\n\tElapsed time is %f s\n', toc);

% Visualize final path
    figure(1);
    hold off;
    imagesc(map);
    hold on;
    plot(path(:,1), path(:,2), 'k:');
    plot(waypoints(:,1), waypoints(:,2), 'g', 'LineWidth', 1.5);
    scatter(waypoints(2:end-1,1), waypoints(2:end-1,2), 'ro', 'MarkerFaceColor', 'r');
    plot(start(1), start(2), 'gs', 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'rd', 'MarkerFaceColor', 'r');
    figure(2)
    hold off;
    imagesc(f);
    colormap('jet');
    colorbar;
    minF = f(path(end,2), path(end, 1));
    caxis([minF*0, max(max(f))]);
    cmap = colormap;
    cmap(1,:) = [0 0 0];
    colormap(cmap);
    
    hold on;
    plot(path(:,1), path(:,2), 'w', 'LineWidth', 2);
    plot(path(:,1), path(:,2), 'k', 'LineWidth', 0.5);
    scatter(waypoints(2:end-1,1), waypoints(2:end-1,2), 'ro', 'MarkerFaceColor', 'r');
    plot(start(1), start(2), 'gs', 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'rd', 'MarkerFaceColor', 'r');
    figure(1);
    
% output final waypoints in inches
waypoints_inch = waypoints * dx;
% Make waypoints relative to starting point
waypoints_inch = waypoints_inch - repmat(waypoints_inch(1, :),size(waypoints_inch,1),1);

dlmwrite('waypoints.txt',waypoints_inch);
end

% Inputs
% map         - Configuration space matrix with obstacles as 1 
% start, goal - Starting and ending positions as [x, y] in map coordinates
% visualize   - Flag for the path finding visualization
function [path, f] = a_star(map, start, goal, visualize)
    % Convert to linear indices
    start_ind = sub2ind(size(map), start(2), start(1));
    goal_ind  = sub2ind(size(map), goal(2), goal(1));
    
    % Initialize data structures
    path = zeros(2,1);
    
    % Open set stores [node, fscore] in each row
    % Maintains order of highest fscore to lowest f score
    open_set = zeros(numel(map),2);
    open_set(1) = start_ind;
    open_len = 1;
    
    % Stores the parent node that led to the current node
    % Used to create the path
    prev = zeros(numel(map), 1);
    
    % g score is the path length from the start to the current node
    g = ones(size(map)) * numel(map);
    g(start_ind) = 0;
    % f score is the g score + the heuristic at the current node
    f = zeros(size(map));
    f(start_ind) = heuristic(map, start_ind, goal_ind);
    
    while open_len > 0
        
        % Gets the lowest f score node on the open set
        [current, open_len] = get_next(open_set, open_len);
                
        % Plots
        if visualize
            figure(1);
            plot_space = map;
            for i = 1:open_len
                plot_space(open_set(i)) = 4;
            end
            plot_space(current) = 5;
            
            current_path = get_path(map, prev, current);
            
            figure(1);
            subplot(1,2,1);
            imagesc(plot_space);
            hold on;
            plot(start(1), start(2), 'go', goal(1), goal(2), 'gx');
            plot(current_path(:,1), current_path(:,2), 'g');
            hold off;
            subplot(1,2,2);
            imagesc(f);
            colorbar;
            hold on;
            plot(start(1), start(2), 'go', goal(1), goal(2), 'gx');
            plot(current_path(:,1), current_path(:,2), 'g');
            hold off;
            drawnow;
        end
        
        % Found the path
        if current == goal_ind
            path = get_path(map, prev, current);
            if visualize
                figure(1);
                subplot(1,2,1);
                imagesc(plot_space);
                subplot(1,2,2);
                imagesc(f);
                colorbar;
            end
            return;
        end
        
        % Mark visited nodes, we do not revisit nodes
        map(current) = 3;
        
        % Get neighbors
        [current_row, current_col] = ind2sub(size(map),current);
        for row = -1:1
            for col = -1:1
                % Eight point connectivity
                if row == 0 && col == 0
                % Four point connectivity
                %if abs(row) == abs(col)
                    continue;
                end
                
                % Check subscripts validity
                if current_row + row > size(map, 1) || current_row + row < 1
                    continue;
                end
                if current_col + col > size(map, 2) || current_col + col < 1
                    continue;
                end
                
                neighbor = sub2ind(size(map), current_row + row, current_col + col);
                
                % Reject obstacle nodes (1) or visited nodes (1)
                if map(neighbor) > 0
                    continue;
                end
                
                % Check if this path is shorter than the previous paths
                new_g = g(current) + dist(map, prev, current, neighbor);
                if new_g < g(neighbor)
                    prev(neighbor) = current;
                    g(neighbor) = new_g;
                    f(neighbor) = g(neighbor) + heuristic(map, neighbor, goal_ind);
                end
                
                % Add the node to the open set
                if ~any(open_set(1:open_len) == neighbor)
                    [open_set, open_len] = add(open_set, open_len, neighbor, f);
                end

            end
        end
    end
    path = 'Failed to find path'
end

% Euclidean distance metric
function distance = dist(map, prev, current, neighbor)
    [y0, x0] = ind2sub(size(map), prev(current));
    [y1, x1] = ind2sub(size(map), current);
    [y2, x2] = ind2sub(size(map), neighbor);
    distance = sqrt((y1-y2)^2 + (x1-x2)^2);
    
    curr_angle = atan2((y2-y1),(x2-x1));
    prev_angle = atan2((y1-y0),(x1-x0));
    if(curr_angle ~= prev_angle)
        distance = distance + 100;
    end
end

% Heuristic is just euclidean distance from node to goal
function h = heuristic(map, node, goal)
    [y, x] = ind2sub(size(map), node);
    [gy, gx] = ind2sub(size(map), goal);
    
    h = sqrt((y-gy)^2 + (x-gx)^2);
end

% Returns an path of [x, y] points in each row, from start to end
function path = get_path(map, prev, current)
    path = zeros(numel(map),2);
    ind = 1;

    while prev(current) ~= 0
        [y, x] = ind2sub(size(map), current);
        path(ind,:) = [x, y];
        current = prev(current);
        ind = ind + 1;
    end
    
    path = path(1:ind-1,:);
    path = flipud(path);
end

% Gets the node with the lowest f value, which is the last item in open_set
function [node, len] = get_next(open_set, len)
    node = open_set(len);
    len = len - 1;
end

% Adds a node to the list, while maintaining descending f score order
function [list, len] = add(list, len, node, f)
    if len == 0
        list(1,:) = [node, f(node)];
        len = len+1;
    else
        for i = len:-1:0
            if i == 0 || f(node) < list(i,2)
                list(i+2:len+2,:) = list(i+1:len+1,:);
                list(i+1,:) = [node, f(node)];
                len = len + 1;
                return
            end
        end
    end
end

% Takes a path and reduces the amount of waypoints, minimizing turns
function [waypoints] = clean_path(path)

waypoints = zeros(size(path, 1), 2);
waypoints(1, :) = path(1,:);
len = 1;
for i = 3:size(path,1)
    point0 = path(i-2, :);
    point1 = path(i-1, :);
    point2 = path(i, :);
    
    prev_ang = atan2(point1(2)-point0(2), point1(1)-point0(1));
    curr_ang = atan2(point2(2)-point1(2), point2(1)-point1(1));

    if angdiff(prev_ang, curr_ang) ~= 0
        len = len + 1;
        waypoints(len,:) = point1;
    end
end
len = len + 1;
waypoints(len, :) = path(end,:);
waypoints = waypoints(1:len, :);

%  % The farthest node that can be reached in a straight line from the
%  % prev node without intersecting obstacles
% farthest_node = 2;
% waypoints = zeros(numel(map),2);
% waypoints(1,:) = path(1,:);
% waypoint_size = 1;
% 
% start_node = 1;
% while(farthest_node < size(path,1))
%     % Check if line between start and end will intersect an obstacle
%     for end_node = (start_node+1):size(path,1)
%         % Use Bresenham's Line Algorithm
%         d = path(end_node,:) - path(start_node,:);
%         
%         if d(1) == 0
%             y0 = path(start_node, 2);
%             y1 = path(end_node, 2);
%             if y1 > y0
%                 y_array = y0:y1;
%             else
%                 y_array = y0:-1:y1;
%             end
%             len = size(y_array', 1);
%             subscripts = zeros(len, 2);
%             subscripts(:,1) = ones(len, 1) * path(start_node, 1);
%             subscripts(:,2) = y_array;
%         else
%             slope = d(2)/d(1);
%             error = 0;
%             subscripts = zeros(numel(map), 2);
%             len = 1;
%             y = path(start_node, 2);
%             if path(start_node, 1) < path(end_node, 1)
%                 x_array = path(start_node, 1):path(end_node,1);
%             else
%                 x_array = path(end_node, 1):-1:path(start_node,1);
%             end
%             for x = x_array
%                 subscripts(len, :) = [y, x];
%                 len = len + 1;
%                 error = error + abs(slope);
%                 while error >= 0.5
%                     y = y + sign(d(2));
%                     error = error - 1;
%                 end
%             end
%             subscripts = subscripts(1:len-1, :);
%         end
%         subscripts
%         inds = sub2ind(size(map), subscripts(:,1), subscripts(:,2));
%         if sum(map(inds)) == 0
%             farthest_node = end_node;
%         end
%     end
%     waypoint_size = waypoint_size + 1;
%     waypoints(waypoint_size,:) = path(farthest_node,:);
%     start_node = farthest_node;
%     farthest_node = start_node+1;
% end
% waypoints = waypoints(1:waypoint_size,:);
end