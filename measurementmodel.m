function measured_values = measurementmodel(nextpose, landmark)
xp = nextpose(1);
yp = nextpose(2);
thetap = nextpose(3);

xl = landmark(1);
yl = landmark(2);

z = sqrt((xl-xp)^2+(yl-yp)^2);
phi = atan2(yl-yp, xl-xp) - thetap;

measured_values = [z, phi];
end