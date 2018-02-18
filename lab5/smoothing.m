function [filtered_waypoints] = smoothing(path, threshold)

x = path(:,1);
y = path(:,2);
% figure;
% plot(x,y)
angles= atan((y(2:end) - y(1:end-1))./(x(2:end)-x(1:end-1)));
% figure;
% plot(angles)

% figure;
smoothed_angles = movmean(angles,2);
% plot (smoothed_angles)

waypoints=[];
for i=1:size(smoothed_angles,1)-1
	if abs(smoothed_angles(i+1) - smoothed_angles(i)) >= threshold
		
        waypoints= [waypoints;[x(i),y(i)]];
	end
end
waypoints = [waypoints;[x(end), y(end)]];

prev_wp = waypoints(1,:);
filtered_waypoints = [prev_wp];

for i=2:size(waypoints,1)
	if norm(waypoints(i,:)-prev_wp)<1
		prev_wp = waypoints(i,:);
		continue;
	end
	filtered_waypoints = [filtered_waypoints;[waypoints(i,:)]];
	prev_wp = waypoints(i,:);

end


% % figure
% % plot(x,y)
% hold on
% for i=1:size(waypoints,1)
% 	scatter(waypoints(i,1), waypoints(i,2))
% 	pause()
% end

% scatter(filtered_waypoints(:,1), filtered_waypoints(:,2))
end