function [quat] = desired_quaternions_values(quat, w, t, ts)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
for k = 1:length(t)-1
   quat(: , k + 1) =  desired_quaternion(quat(:, k), w(:, k), ts);
end
end

