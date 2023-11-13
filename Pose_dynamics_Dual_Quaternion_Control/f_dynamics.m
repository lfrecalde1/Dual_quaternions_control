function [xi_dot] = f_dynamics(p, p_dot, w, force, tau)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%% Parameters System 
m = 100; %% Mass of the system
J_xx = 1.0;
J_yy = 0.93;
J_zz = 0.85;

%% Inertia Matrix
J = [J_xx, 0, 0;...
     0, J_yy, 0;...
     0, 0, J_zz];
%% Auxiliar variable
a = -cross((inv(J)*w), (J*w));

%% F Dual value
F_p = [0; a];
F_d = [0; cross(p, a) + cross(p_dot, w)];

%% U Dual Value
U_p = [0; inv(J)*tau];
U_d = [0; (force/m) + cross(p, inv(J)*tau)];

F = [F_p; F_d];
U = [U_p; U_d];

xi_dot = F + U;
end

