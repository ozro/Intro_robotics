l1 = 3.75; % unit is inch
% Note: theta1 and theta2 are in relative frame

l2 = 2.5; % unit is inch

c_space_matrix = zeros(361,181); % Resolution is 1 degree

t1 = 0:180;
t2 = 0:360;

c_space = ones(181*361,2)*200;  % First col: theta1, second col: theta2. Multiply 200 is for later thresholding

[t1,t2] = meshgrid(t1, t2);


% x = cosd(t1)*l1 + cosd(t1+t2)*l2;
% y = sind(t1)*l1 + sind(t1+t2)*l2;
x = cosd(t1)*l1 + cosd(t2)*l2;
y = sind(t1)*l1 + sind(t2)*l2;
x1 = cosd(t1)*l1;
y1 = sind(t1)*l1;

counter = 1;

x_line = [0,0];
y_line = [0,0];
x_box = [-7,-7,-3,-3,3,3,7,7];
y_box = [0,8,8,5,5,8,8,0];

for row = 1:size(x,1)
    for col = 1:size(x,2)
        
        if(y(row, col) < 0)
            continue
        end
        
        x_line = [x1(row,col),x(row,col)];
        y_line = [y1(row,col),y(row,col)];
         
        [xi,yi] = polyxpoly(x_line,y_line,x_box,y_box);
        
        if (numel(xi) == 0)
            c_space(counter,:) = [t1(1,col) t2(row,1)];
            counter = counter + 1;
            c_space_matrix(row,col) = 1;
        end
    end
end
figure(1);
c_space = c_space(1:counter-1,:);
scatter(c_space(:,1), c_space(:,2), 5,'filled')

figure(2);
x = cosd(c_space(:,1))*l1 + cosd(c_space(:,2))*l2;
y = sind(c_space(:,1))*l1 + sind(c_space(:,2))*l2;
scatter(x,y,'.');
axis([-8,8,-8,8])
c_space_matrix_absolute = flipud(c_space_matrix);
save('c_space_matrix_absolute.mat','c_space_matrix_absolute');