load c_space.mat
targets = [3.5 5; -3.5 5; 6.25 0];

[~, waypoints1] = path_gen(c_space, wavemap, [6.25 0], targets(1,:), false)
[~, waypoints2] = path_gen(c_space, wavemap, targets(1,:), targets(2,:), false)
[~, waypoints3] = path_gen(c_space, wavemap, targets(2,:), targets(3,:), false)

l1 = 3.75;
l2 = 2.5;
waypoints = [waypoints1;waypoints2;waypoints3;];
xy = [cos(waypoints(:,1))*l1 + cos(waypoints(:,2)) * l2,...
      sin(waypoints(:,1))*l1 + sin(waypoints(:,2)) * l2];

for i = 2:size(xy,1)
    xy(i,:) = xy(i-1,:) + xy(i,:);
end

figure(3);
plot(xy);
hold on;
p1 = [-3 3 3 -3];
p2 = [5 5 8 8];
fill(p1,p2,'r')
axis([-7 7 0 8]);

waypoints1(:,1)'
waypoints2(:,1)'
waypoints3(:,1)'
waypoints1(:,2)'
waypoints2(:,2)'
waypoints3(:,2)'