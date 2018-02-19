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
waypoints = clean_path(map, path);
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
waypoints_inch(:, 2) = 48*ones(size(waypoints_inch(:,2))) - waypoints_inch(:, 2);
waypoints_inch = waypoints_inch - repmat(waypoints_inch(1, :), size(waypoints_inch,1), 1);
waypoints_inch = waypoints_inch(2:end-1,:);
dlmwrite('waypoints.txt',waypoints_inch' * 25.4);
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
        distance = distance + 1;
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
function [waypoints] = clean_path(map, path)
start_ind = 1;
farthest_feasible = 2;
num_points = 100;
cleaned_path = zeros(1000,2);
cleaned_path(1,:) = path(start_ind,:);
cleaned_path_size = 1;
hitting_threshold = 1;

while(farthest_feasible < size(path,1))
    for next_ind = (start_ind+1):size(path,1)
        path_sample_x = linspace(path(start_ind,1), path(next_ind,1), num_points);
        path_sample_y = linspace(path(start_ind,2), path(next_ind,2),num_points);
        path_sample = [path_sample_y', path_sample_x'];
        if sum(map(sub2ind(size(map), int64(path_sample(:,1)), int64(path_sample(:,2))))) < hitting_threshold
            farthest_feasible = next_ind;
%         else
%             sum(map(sub2ind(size(map), int64(path_sample(:,1)), int64(path_sample(:,2)))))
%             cleaned_path(cleaned_path_size+1,:) = filtered_waypoints(next_ind-1,:);
%             cleaned_path_size = cleaned_path_size + 1;
%             start_ind = next_ind-1;
        end

    end
                 cleaned_path(cleaned_path_size+1,:) = path(farthest_feasible,:);
             cleaned_path_size = cleaned_path_size + 1;
             start_ind = farthest_feasible;
             farthest_feasible = start_ind+1;
    
end
cleaned_path(cleaned_path_size+1,:) = path(end,:);
cleaned_path_size = cleaned_path_size + 1;
waypoints = cleaned_path(1:cleaned_path_size,:);
end