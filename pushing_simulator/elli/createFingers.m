function [ep_lf, ep_rf] = createFingers(Ra, Rb, a, l_lf, l_rf, v_p, d, psi)

x_d = -d;
y_d = -sqrt((1-x_d^2/Ra^2)*Rb^2);
contact_pt = [x_d; y_d];

ep_lf = [contact_pt contact_pt-v_p*l_lf/norm(v_p)]; %two end points of the left finger
ep_rf = [0  0;
         0  0];
ep_rf(1,2) = ep_lf(1,2) + a*sin(psi);
ep_rf(2,2) = ep_lf(2,2) + a*cos(psi);
ep_rf(:,1) = ep_rf(:,2) + v_p*l_rf/norm(v_p);

end