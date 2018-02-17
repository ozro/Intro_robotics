% Inputs:
% map         - matrix with obstacles as 1 and free space as 0
% dx          - number of inches per element in discretized map
% start, goal - [x, y] of starting and ending positions, in inches
% visualize   - flag for path finding process visualization
function path_gen(start, goal, visualize)
tic;
map = load('map.mat');
map = map.map;
dx = 0.014;

% Change to map coorinates
start = round(start/dx);
goal = round(goal/dx);

% Define robot radius for configuration space
r = 0.2/dx;

% Convert to configuration space
map = config_space(map, r);

% Calculate path using A*
path = a_star(map, start, goal, visualize);

toc

if visualize
    figure(1);
    subplot(1,2,1);
    hold on;
    plot(path(:,1), path(:,2), 'r');
    hold off;
    subplot(1,2,2);
    hold on;
    plot(path(:,1), path(:,2), 'r');
    hold off;
else
    figure(1);
    imagesc(map);
    hold on;
    plot(path(:,1), path(:,2), 'g');
    plot(start(1), start(2), 'ro', goal(1), goal(2), 'rx');
    hold off;
end
end

% Converts map to configuration space using robot radius r
function config = config_space(map, r)
    config = map;
end

% Inputs
% map         - Configuration space matrix with obstacles as 1 
% start, goal - Starting and ending positions as [x, y] in map coordinates
% visualize   - Flag for the path finding visualization
function path = a_star(map, start, goal, visualize)
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
                plot_space(open_set(i)) = 3;
            end
            plot_space(current) = 4;
            figure(1);
            subplot(1,2,1);
            imagesc(plot_space);
            hold on;
            plot(start(1), start(2), 'ro', goal(1), goal(2), 'rx');
            hold off;
            subplot(1,2,2);
            imagesc(f);
            colorbar;
            hold on;
            plot(start(1), start(2), 'ro', goal(1), goal(2), 'rx');
            hold off;
            drawnow;
        end
        
        % Found the path
        if current == goal_ind
            path = get_path(map, prev, current);
            return;
        end
        
        % Mark visited nodes, we do not revisit nodes
        map(current) = 2;
        
        % Get neighbors
        [current_row, current_col] = ind2sub(size(map),current);
        for row = -1:1
            for col = -1:1
                % Four point connectivity
                if abs(row) == abs(col)
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
                new_g = g(current) + dist(map, current, neighbor);
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
        
        path = 'Failed to find path';
    end
end

% Euclidean distance metric
function d = dist(map, current, neighbor)
    [y1, x1] = ind2sub(size(map), current);
    [y2, x2] = ind2sub(size(map), neighbor);
    d = sqrt((y1-y2)^2 + (x1-x2)^2);
end

% Heuristic is just euclidean distance from node to goal
function h = heuristic(map, node, goal)
    [y, x] = ind2sub(size(map), node);
    [gy, gx] = ind2sub(size(map), goal);
    
    h = 2*sqrt((y-gy)^2 + (x-gx)^2);
end

% Returns an path of [x, y] points in each row
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