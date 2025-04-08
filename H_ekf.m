function H = H_ekf(current_pose,landmark)
x = current_pose(1);
y = current_pose(2);
% theta = current_pose(3);

xl = landmark(1);
yl = landmark(2);

r = sqrt((x-xl)^2+(y-yl)^2);

H = [(x-xl)/r, (y-yl)/r, 0; -(y-yl)/(r^2), (x-xl)/(r^2), -1];

end