function [obj_pts] = createCircleObject(x, y, r)

th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
obj_pts = [xunit; yunit];

end