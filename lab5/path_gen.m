% Inputs must be [x, y] of starting and ending positions, in m
function path_gen(start, goal)
% Define map dimensions in m
map_w = 3;
map_h = 3;

% Define quanta of distance for discretization in m
dx = 0.01;

% Generate map space
map = zeros(map_h/dx, map_w/dx);

% Add obstacles
map(1.5/dx:1.75/dx, 1.5/dx:1.75/dx) = 1;

% Define robot dimensions
r = 0.2/dx;

% Convert to configuration space
map = config_space(map, r);

% Calculate path
path = find_path(map, start, goal);

% Display map and path
imagesc(map);
hold on;

% Draw start and goal positions
plot(start(1), start(2), 'go');
plot(goal(1), goal(2), 'rx');
plot(1:size(path, 1), path, 'r-');
hold off;
end

function config = config_space(map, r)
    config = map;
end

function path = find_path(map, start, goal)
    path = zeros(2,1);
end