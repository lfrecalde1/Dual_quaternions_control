function [x] = desired_quaternion(x, M, ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = quat_dot(x, M);
k2 = quat_dot(x + ts/2*k1, M); % new
k3 = quat_dot(x + ts/2*k2, M); % new
k4 = quat_dot(x + ts*k3, M); % new

x = x +ts/6*(k1 +2*k2 +2*k3 +k4); % new

end