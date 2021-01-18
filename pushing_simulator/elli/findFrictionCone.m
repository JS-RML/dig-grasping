% Input:
% fc_angle: angle between the contact normal and one edge of the friction
% cone
% d: distance between the contact point and the object minor axis
% Output:
% F_l: left edge of the friction wrench cone
% F_r: right edge of the friction wrench cone
function [F_l, F_r, contact_pt, normalized_n] = findFrictionCone(object_type, Ra, Rb, fc_angle, f_p, d)

if (strcmp(object_type, 'ellipse'))
    x_d = -d;
    y_d = -sqrt((1-x_d^2/Ra^2)*Rb^2);
    
    n = [2*x_d/Ra^2;2*y_d/Rb^2];
    normalized_n = -bsxfun(@rdivide, n, sqrt(sum(n.^2))); % inward normal vector
elseif (strcmp(object_type, 'circle'))
    x_d = -d;
    y_d = -sqrt(Ra^2-x_d^2);
    n = [x_d; y_d];
    normalized_n = -bsxfun(@rdivide, n, sqrt(sum(n.^2))); % inward normal vector
end
contact_pt = [x_d y_d]';
n_visual = contact_pt + normalized_n;
 
% Calculate friction force f_l and f_r
R_l = [cos(fc_angle) -sin(fc_angle);
     sin(fc_angle) cos(fc_angle)];
f_l = R_l * (normalized_n * f_p);
f_l_visual = contact_pt + f_l;
% f_l = R_l * (normalized_n_real * f_p);
% f_l_visual = contact_pt_real + f_l;

R_r = [cos(-fc_angle) -sin(-fc_angle);
     sin(-fc_angle) cos(-fc_angle)];
f_r = R_r * (normalized_n * f_p);
f_r_visual = contact_pt + f_r;
% f_r = R_r * (normalized_n_real * f_p);
% f_r_visual = contact_pt_real + f_r;

% Calculate moment (about COM)
dist_l = abs(det([f_l_visual-contact_pt,[0;0]-contact_pt]))/norm(f_l_visual-contact_pt); % distance from COM to the left friction edge
dist_r = abs(det([f_r_visual-contact_pt,[0;0]-contact_pt]))/norm(f_r_visual-contact_pt);
mom_l = f_p * dist_l;
mom_r = f_p * dist_r;

% F_l = [f_l; mom_l];
% F_r = [f_r; mom_r];

Jp = [1 0 -contact_pt(2);
      0 1  contact_pt(1)];
F_l = Jp' * f_l;
F_r = Jp' * f_r;

% Visualize the distance from COM to the edge of friction cone
% % dist_l_vec = normalized_n * dist_l;
% % R = [cos(pi/2+fc_angle) -sin(pi/2+fc_angle);
% %      sin(pi/2+fc_angle) cos(pi/2+fc_angle)];
% % dist_l_vec = R * dist_l_vec;
% % dist_l_visual = [0 0]' + dist_l_vec;
% % 
% % dist_r_vec = normalized_n * dist_r;
% % R = [cos(-pi/2-fc_angle) -sin(-pi/2-fc_angle);
% %      sin(-pi/2-fc_angle) cos(-pi/2-fc_angle)];
% % dist_r_vec = R * dist_r_vec;
% % dist_r_visual = [0 0]' + dist_r_vec;
% % 
% theta = 0 : 0.01 : 2*pi;
% x_ellipse = Ra * cos(theta);
% y_ellipse = Rb * sin(theta);
% Pts = [x_ellipse; y_ellipse];
% 
% figure;
% plot(x_ellipse, y_ellipse, 'LineWidth', 3);
% hold on;
% line([contact_pt(1) n_visual(1)], [contact_pt(2) n_visual(2)], 'LineWidth', 2, 'Color', [1,0,0]);
% line([contact_pt(1) f_l_visual(1)], [contact_pt(2) f_l_visual(2)], 'LineWidth', 2, 'Color', [1,0,0]);
% line([contact_pt(1) f_r_visual(1)], [contact_pt(2) f_r_visual(2)], 'LineWidth', 2, 'Color', [1,0,0]);
% % line([0 dist_l_visual(1)], [0 dist_l_visual(2)], 'LineWidth', 2);
% % line([0 dist_r_visual(1)], [0 dist_r_visual(2)], 'LineWidth', 2);
% axis square;
% xlim([-0.1 0.1]);
% ylim([-0.1 0.1]);
% grid on;

% Visualize for circle
% xCenter = 0;
% yCenter = 0;
% [obj_pts] = createCircleObject(xCenter, yCenter, Ra);
% 
% figure;
% plot(obj_pts(1,:), obj_pts(2,:), 'LineWidth', 3);
% hold on;
% line([contact_pt(1) n_visual(1)], [contact_pt(2) n_visual(2)], 'LineWidth', 2, 'Color', [1,0,0]);
% line([contact_pt(1) f_l_visual(1)], [contact_pt(2) f_l_visual(2)], 'LineWidth', 2, 'Color', [1,0,0]);
% line([contact_pt(1) f_r_visual(1)], [contact_pt(2) f_r_visual(2)], 'LineWidth', 2, 'Color', [1,0,0]);
% % line([0 dist_l_visual(1)], [0 dist_l_visual(2)], 'LineWidth', 2);
% % line([0 dist_r_visual(1)], [0 dist_r_visual(2)], 'LineWidth', 2);
% axis square;
% xlim([-0.1 0.1]);
% ylim([-0.1 0.1]);
% grid on;

end