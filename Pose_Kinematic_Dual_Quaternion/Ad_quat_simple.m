function [Ad] = Ad_quat_simple(q_e, xi_d)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
q_e_c = congujate_quaternion(q_e);
ad_aux = quaternion_multiply(q_e, xi_d);
Ad = quaternion_multiply(ad_aux, q_e_c);

end