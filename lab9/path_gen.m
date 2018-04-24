% Inputs:
% map         - configuration space with obstacles as 1 and free space as 0
% wavemap     - map wavefronts propagated from obstacles
% dx          - number of inches per element in discretized map
% start, goal - [x, y] of starting and ending positions, in degrees
% visualize   - flag for path finding process visualization

% Outputs:
% waypoints         - a list of waypoints relative to the starting position
% waypoints_inch    - a list of waypoints in inches
% waypoints.txt     - a text file containing waypoinst in millimeters
function [waypoints, waypoints_relative] = path_gen(map, wavemap, start, goal, visualize)
x = start(1);
y = start(2);
l1 = 3.75;
l2 = 2.5;
Theta2 = acos((x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2));
s1 = (y*(l1 + l2*cos(Theta2)) - x*l2*sin(Theta2))/(x^2+y^2);
c1 = (x+s1*(l2*sin(Theta2)))/(l1+l2*cos(Theta2));

Theta1 = atan2(s1,c1);
if(Theta1 < 0 || Theta1 > pi)
    alpha = atan2(y,x);
    Theta1 = (alpha - Theta1) + alpha;
    Theta2 = -Theta2;
end
start = [Theta1 Theta2+Theta1] * 180 / pi;

x = goal(1);
y = goal(2);
l1 = 3.75;
l2 = 2.5;
Theta2 = acos((x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2));
s1 = (y*(l1 + l2*cos(Theta2)) - x*l2*sin(Theta2))/(x^2+y^2);
c1 = (x+s1*(l2*sin(Theta2)))/(l1+l2*cos(Theta2));

Theta1 = atan2(s1,c1);
if(Theta1 < 0 || Theta1 > pi)
    alpha = atan2(y,x);
    Theta1 = (alpha - Theta1) + alpha;
    Theta2 = -Theta2;
end

goal = [Theta1 Theta2+Theta1] * 180/pi;

start = round(start * 2)+1;
goal = round(goal * 2)+1;
start(2) = mod(start(2) + 360, 720);
goal(2) = mod(goal(2) + 360,720);
% Change map to uint8
map = uint8(map);

% Calculate path using A*
fprintf('Running A* algorithm...');
tic
[path,f] = a_star(map, wavemap, start, goal, visualize);
fprintf(' Complete!\n\tElapsed time is %f s\n\n', toc);

% Refine the path to key waypoints
fprintf('Running path cleaner...');
tic
waypoints = clean_path(path);
fprintf(' Complete!\n\tElapsed time is %f s\n', toc);


% Visualize final path
close all;
    figure(1);
    hold off;
    imagesc(map);
    hold on;
    plot(path(:,1), path(:,2), 'k:');
    plot(waypoints(:,1), waypoints(:,2), 'g', 'LineWidth', 1.5);
    scatter(waypoints(2:end-1,1), waypoints(2:end-1,2), 'ro', 'MarkerFaceColor', 'r');
    plot(start(1), start(2), 'gs', 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'rd', 'MarkerFaceColor', 'r');
    
    figure(2);
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
    
% output final waypoints in relative
waypoints = (waypoints - 1)/2;
%waypoints(1,:) = [0, 0];
for i = 2:size(waypoints, 1)
    waypoints_relative(i-1, :) = waypoints(i,:) - waypoints(i-1,:);
end
waypoints = waypoints_relative;
waypoints_relative = waypoints_relative * [5 0; 0 40/24];
end

% Inputs
% map         - Configuration space matrix with obstacles as 1 
% start, goal - Starting and ending positions as [x, y] in map coordinates
% visualize   - Flag for the path finding visualization
function [path, f] = a_star(map, wavemap, start, goal, visualize)
    % Convert to linear indices
    start
    goal
    start_ind = sub2ind(size(map), start(2), start(1));
    goal_ind  = sub2ind(size(map), goal(2), goal(1));
    
    % Initialize data structures
    path = zeros(2,1);
    
    % Calculate maximum separation from obstacles
    wavepeak = max(max(wavemap));
    
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
    f(start_ind) = heuristic(map, wavemap, wavepeak, start_ind, goal_ind);
    
    while open_len > 0
        
        % Gets the lowest f score node on the open set
        [current, open_len] = get_next(open_set, open_len);
                
        % Plots
        if visualize
            figure(3);
            plot_space = map;
            for j = 1:open_len
                plot_space(open_set(j)) = 4;
            end
            plot_space(current) = 5;
            
            current_path = get_path(map, prev, current);
            
            figure(2);
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
                %if row == 0 && col == 0
                % Four point connectivity
                if abs(row) == abs(col)
                    continue;
                end
                
                % Check subscripts validity
                if current_row + row > size(map, 1) ||  current_row + row < 1
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
                new_g = g(current) + dist(map, wavemap, wavepeak, prev, current, neighbor);
                if new_g < g(neighbor)
                    prev(neighbor) = current;
                    g(neighbor) = new_g;
                    f(neighbor) = g(neighbor) + heuristic(map, wavemap, wavepeak, neighbor, goal_ind);
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
function distance = dist(map, wavemap, peak, prev, current, neighbor)
    [y0, x0] = ind2sub(size(map), prev(current));
    [y1, x1] = ind2sub(size(map), current);
    [y2, x2] = ind2sub(size(map), neighbor);
    %distance = sqrt((y1-y2)^2 + (x1-x2)^2)/2;
    distance = abs(y1-y2) + abs(x1-x2);
    % Weight paths by distance to closest obstacle
    distance = distance + (peak-wavemap(current))/2;

    % Weight paths by number of turns
    curr_angle = atan2((y2-y1),(x2-x1));
    prev_angle = atan2((y1-y0),(x1-x0));
    diff = abs(angdiff(curr_angle, prev_angle));
    if(diff > 0)
        distance = distance + 100;
    end
end

% Heuristic is just euclidean distance from node to goal
function h = heuristic(map, wavemap, peak, node, goal)
    [y, x] = ind2sub(size(map), node);
    [gy, gx] = ind2sub(size(map), goal);
    
    h = abs(y-gy) + abs(x-gx) + (peak-wavemap(node));

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
    
    % Make path go from start to finish
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
        for j = len:-1:0
            if j == 0 || f(node) < list(j,2)
                list(j+2:len+2,:) = list(j+1:len+1,:);
                list(j+1,:) = [node, f(node)];
                len = len + 1;
                return
            end
        end
    end
end

% Takes a path and pick waypoints where the angle of the path changes
function [waypoints] = clean_path(path)
waypoints = zeros(size(path));
len = 1;
waypoints(len, :) = path(1,:);
for j = 3:size(path, 1)-1
    p0 = path(j - 1, :);
    p1 = path(j,:);
    p2 = path(j+1, :);
    prev_angle = atan2((p1(2)-p0(2)),(p1(1)-p0(1)));
    curr_angle = atan2((p2(2)-p1(2)),(p2(1)-p1(1)));
    diff = abs(angdiff(curr_angle, prev_angle));
    if(diff > 0)
        len = len + 1;
        waypoints(len,:) = p1;
    end
end
len = len + 1;
waypoints(len, :) = path(end, :);
waypoints = waypoints(1:len, :);
end