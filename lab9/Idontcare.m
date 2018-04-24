load c_space.mat
targets = [-5 1; 3.5 5; 6.25 0];

[~, waypoints1] = path_gen(c_space, wavemap, [6.25 0], targets(1,:), false)
[~, waypoints2] = path_gen(c_space, wavemap, targets(1,:), targets(2,:), false)
[~, waypoints3] = path_gen(c_space, wavemap, targets(2,:), targets(3,:), false)

len = max([size(waypoints1,1), size(waypoints2,1), size(waypoints3,1)])

waypoints1a = [waypoints1', zeros(2, len - size(waypoints1,1))];
waypoints2a = [waypoints2', zeros(2, len - size(waypoints2,1))];
waypoints3a = [waypoints3', zeros(2, len - size(waypoints3,1))];


waypoints1a(1,:)
waypoints2a(1,:)
waypoints3a(1,:)

waypoints1a(2,:)
waypoints2a(2,:)
waypoints3a(2,:)

format = repmat('%.1f,', 1, len);
format = ['{' format(:, 1:end-1) '}'];
format = repmat([format ','], 1, 3);
format = ['{' format(:, 1:end-1) '};\n'];
fprintf(['int wpth1[3][%d] =' format], [len waypoints1a(1,:) waypoints2a(1,:) waypoints3a(1,:)])
fprintf(['int wpth2[3][%d] =' format], [len waypoints1a(2,:) waypoints2a(2,:) waypoints3a(2,:)])