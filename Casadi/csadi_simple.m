clc, clear all, close all;

%% 
addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;

x = SX.sym('x'); 
y = SX.sym('y');
z = SX.sym('z');

R = eye(3);

optimization_variables = [x; y; z];

f = 1/2*(optimization_variables'*R*optimization_variables);

Jacobiano = jacobian(f,optimization_variables);

Hessiano = hessian(f, optimization_variables);

f_eval = Function('f_eval',{optimization_variables},{f});

aux = f_eval([2;3;5])

J_eval = Function('J_eval',{optimization_variables},{Jacobiano});

aux_2 = J_eval([2;3;5])