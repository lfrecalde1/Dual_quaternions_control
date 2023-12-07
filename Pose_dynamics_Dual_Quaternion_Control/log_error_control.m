function [log_he] = log_error_control(h_d, h)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
h_d_c(:, 1) = conjugate_dual(h_d(:, 1));
he(:, 1) = mult_dual(h(:, 1),h_d_c(:, 1));
log_he = log_error(he(:, 1));

%% Compute quaternion error
p_d = get_traslatation_dual(h_d);
R_d = get_rotation_dual(h_d);

p = get_traslatation_dual(h);
R = get_rotation_dual(h);

%% Compute error quaternion rotational
r = R(1);
v = R(2:4);

r_d = R_d(1);
v_d = R_d(2:4);

e_quat = [r*r_d + v'*v_d;...
          r*v_d - r_d*v - hat_map(v)*v_d];
      
aux = e_quat(1)^2 + e_quat(2:4)'*e_quat(2:4);

end

