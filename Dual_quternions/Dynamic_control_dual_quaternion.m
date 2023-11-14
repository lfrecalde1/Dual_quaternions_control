function [U] = Dynamic_control_dual_quaternion(h_d, h, xi_d, xi, p, p_dot, w, parameters, kp, kd)
%UNTITLED11 Summary of this function goes here
%% Gains
kp_pose = kp*[0;1;1;1];
kp_attitude = kp*[0;1;1;1];
kp = [kp_pose; kp_attitude];

kd_pose = kd*[0;1;1;1];
kd_attitude = kd*[0;1;1;1];
kd = [kd_pose; kd_attitude];

%% Feedforward
%% Parameters System
m = parameters(1); %% Mass of the system
J_xx = parameters(2);
J_yy = parameters(3);
J_zz = parameters(4);

%% Inertia Matrix
J = [J_xx, 0, 0;...
    0, J_yy, 0;...
    0, 0, J_zz];
%% Auxiliar variable
a = -cross((inv(J)*w), (J*w));

%% F Dual value
F_p = [0; a];
F_d = [0; cross(p, a) + cross(p_dot, w)];
F = [F_p; F_d];
%   Detailed explanation goes here
[log_he] = log_error_control(h_d(:, 1), h(:, 1));
[velocity_he] = velocity_error(h_d(:, 1), h(:, 1), xi_d(:, 1), xi(:, 1));

h_d_c(:, 1) = conjugate_dual(h_d(:, 1));
he(:, 1) = mult_dual(h(:, 1),h_d_c(:, 1));

aux_ad = Ad_quat(he(:, 1), xi_d);

U = -2*inner_product_dual_vector_quaternion(kp, log_he) - inner_product_dual_vector_quaternion(kd, velocity_he)- F + (1/2)*mult_dual(velocity_he, aux_ad) - (1/2)*mult_dual(aux_ad, velocity_he);

end

