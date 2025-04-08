function A = A_ekf(current_pose,input,t)
% x = current_pose(1);
% y = current_pose(2);
theta = current_pose(3);
v = input(1);
omega = input(2);

vbyomega = v/omega;

A = [1,0,vbyomega*(cos(omega*t+theta)-cos(theta));0,1,vbyomega*(sin(omega*t+theta)-sin(theta));0,0,1];


end