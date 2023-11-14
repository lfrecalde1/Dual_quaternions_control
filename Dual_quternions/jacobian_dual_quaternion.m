function [dual_quaternion_J] = jacobian_dual_quaternion(p_dot, p, w)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
primal = [0; w];
dual = [0;p_dot + cross(p, w)];
dual_quaternion_J = [primal; dual];
end

