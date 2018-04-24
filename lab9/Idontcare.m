load c_space.mat
targets = [2 1; -2 1; 6.25 0];

[~, waypoints1] = path_gen(c_space, wavemap, [6.25 0], targets(1,:), false)
[~, waypoints2] = path_gen(c_space, wavemap, targets(1,:), targets(2,:), false)
[~, waypoints3] = path_gen(c_space, wavemap, targets(2,:), targets(3,:), false)

l1 = 3.75;
l2 = 2.5;
waypoints = [waypoints1;waypoints2;waypoints3;];

waypoints1(:,1)'
waypoints2(:,1)'
waypoints3(:,1)'
waypoints1(:,2)'
waypoints2(:,2)'
waypoints3(:,2)'