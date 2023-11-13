function [velocity_error] = velocity_error(h_d, h, xi_d, xi)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
h_d_c(:, 1) = conjugate_dual(h_d(:, 1));
he(:, 1) = mult_dual(h(:, 1),h_d_c(:, 1));

velocity_error = xi - Ad_quat(he(:, 1), xi_d);
end

