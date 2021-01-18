function [obj_pts] = createEllipticalObject(xCenter, yCenter, Ra, Rb)

theta = 0 : 0.01 : 2*pi;
x_ellipse = Ra * cos(theta) + xCenter;
y_ellipse = Rb * sin(theta) + yCenter;
obj_pts = [x_ellipse; y_ellipse];

end