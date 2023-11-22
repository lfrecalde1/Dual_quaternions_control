function [Ad] = Ad_quat(q, F)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
F = [0; F];
q_e_c = congujate_quaternion(q);
ad_aux = quaternion_multiply(q, F);
Ad = quaternion_multiply(ad_aux, q_e_c);
Ad = Ad(2:4);
end