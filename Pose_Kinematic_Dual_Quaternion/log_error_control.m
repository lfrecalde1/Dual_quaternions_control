function [log_he] = log_error_control(h_d, h)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
h_d_c(:, 1) = conjugate_dual(h_d(:, 1));
he(:, 1) = mult_dual(h(:, 1),h_d_c(:, 1));
log_he = log_error(he(:, 1));
end

