function [h] = inner_product_dual_vector_quaternion(k, v)
%UNTITLED17 Summary of this function goes here
%   Detailed explanation goes here
kp_pose = k(2:4);
kp_attitude = k(6:8);

v_primal = v(2:4);
v_dual = v(6:8);
primal = [0; diag(kp_pose)*v_primal];
dual = [0; diag(kp_attitude)*v_dual];

h = [primal; dual];
end

