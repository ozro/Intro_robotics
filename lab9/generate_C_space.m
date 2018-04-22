l1 = 3.75; % unit is inch
% Note: theta1 and theta2 are in relative frame

l2 = 2.5; % unit is inch

c_space_idx = ones(180,360); % Resolution is 1 degree

t1 = 0:180;
t2 = -180:180;

c_space = ones(180*360,2)*200;  % First col: theta1, second col: theta2. Multiply 200 is for later thresholding

[t1,t2] = meshgrid(t1, t2);

x = cosd(t1)*l1 + cosd(t1+t2)*l2;
y = sind(t1)*l1 + sind(t1+t2)*l2;
x1 = cosd(t1)*l1;
y1 = sind(t1)*l1;

counter = 1;
for row = 1:size(x,1)
    for col = 1:size(x,2)
         N = 10;   % Number of points on the line to test
        [xx,yy] = fill_line([x1(row,col), y1(row,col)], [x(row,col) y(row,col)], N);
        valid = 1;
        
        for j = 1:length(xx)
            if ((xx(j) >= -3.5 && xx(j) <= 3.5 && yy(j) >= 4.5) || ...
                (abs(xx(j)) > 7) || ...
                (yy(j) > 8) || ...
                (yy(j) < 0))
             
                valid = 0;
                break;
            end
        end
        
        if (valid == 1)
            c_space(counter,:) = [t1(1,col) t2(row,1)];
            counter = counter + 1;
        end
    end
end

c_space = c_space(1:counter-1,:);
scatter(c_space(:,1), c_space(:,2), 5,'filled')