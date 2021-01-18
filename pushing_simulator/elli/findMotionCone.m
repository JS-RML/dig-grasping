function[v_l, v_r, V_l, V_r] = findMotionCone(F_l, F_r, A, contact_pt)

V_l = F_l'*(A+A'); % twist resulting from left wrench cone
V_l = V_l';
% V_l = bsxfun(@rdivide, V_l, sqrt(sum(V_l.^2))); % unify the vector
V_r = F_r'*(A+A');
V_r = V_r';
% V_r = bsxfun(@rdivide, V_r, sqrt(sum(V_r.^2)));
Jp = [1 0 -contact_pt(2);
      0 1  contact_pt(1)];
v_l = Jp * V_l; % left motion cone
v_r = Jp * V_r;

end