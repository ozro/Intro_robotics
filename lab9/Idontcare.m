x = -3;
y = 3;
l1 = 3.75;
l2 = 2.5;
Beta = acos((x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2));
s1 = (y*(l1 + l2*cos(Beta)) - x*l2*sin(Beta))/(x^2+y^2);
c1 = (x+s1*(l2*sin(Beta)))/(l1+l2*cos(Beta));

Theta1 = 180*atan2(s1,c1)/pi;

Theta2 = 180*Beta/pi + Theta1;
