function [obj_pts] = createRecObject(xCenter, yCenter, Ra, Rb)

left_bottom_corner = [xCenter-Ra/2; yCenter-Rb/2];
left_up_corner = [xCenter-Ra/2; yCenter+Rb/2];
right_bottom_corner = [xCenter+Ra/2; yCenter-Rb/2];
right_up_corner = [xCenter+Ra/2; yCenter+Rb/2];
obj_pts = [left_bottom_corner, right_bottom_corner, right_up_corner, left_up_corner, left_bottom_corner];

end