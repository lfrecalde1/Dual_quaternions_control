function [output_aux] = quaternion_multiply(q1, q2)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

%% Better formulation
Q1_aux = matrix_q(q1);

Q_aux = Q1_aux*q2;

output_aux= Q_aux;
end
