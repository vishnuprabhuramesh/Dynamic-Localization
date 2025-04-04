function nextpose = velocitymodel(pose, input, t)
x = pose(1);
y = pose(2);
theta = pose(3);

v = input(1);
omega = input(2);

if omega == 0
    xp = x + v*t*cos(theta);
    yp = y + v*t*sin(theta);
    thetap = theta;
else
    xp = x + (v/omega)*(sin(omega*t+theta)-sin(theta));
    yp = y - (v/omega)*(cos(omega*t+theta)-cos(theta));
    thetap = omega*t + theta;
end
nextpose = [xp, yp, thetap];

end