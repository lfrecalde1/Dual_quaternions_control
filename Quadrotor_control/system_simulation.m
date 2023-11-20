function [x] = system_simulation(x, F, M, L, ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = system_dynamics(x, F, M,  L);
k2 = system_dynamics(x + ts/2*k1, F, M, L); % new
k3 = system_dynamics(x + ts/2*k2, F, M, L); % new
k4 = system_dynamics(x + ts*k3, F, M, L); % new

x = x +ts/6*(k1 +2*k2 +2*k3 +k4); % new

end