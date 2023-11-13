function [Ad] = Ad_quat(q_e, xi_d)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
q_e_c = conjugate_dual(q_e);
ad_aux = mult_dual(q_e, xi_d);
Ad = mult_dual(ad_aux, q_e_c);

end

