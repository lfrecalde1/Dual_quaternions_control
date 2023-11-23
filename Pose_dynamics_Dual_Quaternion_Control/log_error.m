function [log_he] = log_error(h)
%UNTITLED12 Summary of this function goes here
%   Detailed explanation goes here
if h(1)>=0
   h = 1*h;
else
   h = -1*h;
end
p = get_traslatation_dual(h);
r = get_rotation_dual(h);
[Angle_axis(:, 1)] = quat2axang(r');
[value] = quaternionToAxisAngle(r);

log_r = [0;(1/2)*(value)];
log_t = [0; (1/2)*p(2:4)];

log_he = [log_r;log_t];
end

