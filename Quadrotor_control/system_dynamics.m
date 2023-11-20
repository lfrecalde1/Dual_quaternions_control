function [xp] = system_dynamics(x, F, M, L)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%% Parameters
m = L(1);
Jxx = L(2);
Jyy = L(3);
Jzz = L(4);
gravity = L(5);

%% Inertial Matrix
J = [Jxx, 0, 0;...
     0, Jyy, 0;...
     0, 0, Jzz];

%% Auxiliar variable
e3 = [0; 0; 1];
g = gravity * e3;
%% Split values
vel = x(4:6);
quat = x(7:10);
omega = x(11:13);
R = QuatToRot(quat');


%% Evolution of the system
qdot = quat_dot(quat, omega);
acc = ((F*(R*e3))/m)  - g;
aux = J*omega;
omega_dot = inv(J)*(M - cross(omega, aux));

%% Output of the system
xp = [vel;...
      acc;...
      qdot;...
      omega_dot];
end

