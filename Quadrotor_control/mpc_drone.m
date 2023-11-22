function [f,solver,args] = mpc_drone(bounded, N, L, ts)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;

%% Definicion de las restricciones en las acciones de control
F_max = bounded(1); 
F_min = bounded(2);

tau_1_max = bounded(3);
tau_1_min = bounded(4);

tau_2_max = bounded(5);
tau_2_min = bounded(6);

tau_3_max = bounded(7); 
tau_3_min = bounded(8);

%% Generacion de las variables simbolicas de los estados del sistema
x = SX.sym('x'); 
y = SX.sym('y');
z = SX.sym('z');

vx = SX.sym('vx'); 
vy = SX.sym('vy');
vz = SX.sym('vz');

qw = SX.sym('qw');
q1 = SX.sym('q1');
q2 = SX.sym('q2');
q3 = SX.sym('q3');

wx = SX.sym('wx');
wy = SX.sym('wy');
wz = SX.sym('wz');

%% Definicion de cuantos estados en el sistema
states = [x; y; z; vx; vy; vz; qw; q1; q2; q3; wx; wy; wz];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
F_ref = SX.sym('F_ref');
tau_1_ref = SX.sym('tau_1_ref');
tau_2_ref = SX.sym('tau_2_ref');
tau_3_ref = SX.sym('tau_3_ref');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [F_ref; tau_1_ref; tau_2_ref; tau_3_ref]; 
n_control = length(controls);

%% System parameters
m = L(1);
Jxx = L(2);
Jyy = L(3);
Jzz = L(4);
gravity = L(5);

%% Inertial Matrix
J = [Jxx, 0, 0;...
     0, Jyy, 0;...
     0, 0, Jzz];

%% Dynamics of the system 

%% Auxiliar variable
e3 = [0; 0; 1];
g = gravity * e3;

%% Split values
vel = states(4:6);
quat = states(7:10);
omega = states(11:13);
R = QuatToRot_casadi(quat');

qdot = quat_dot(quat, omega);
acc = ((F_ref*(R*e3))/m)  - g;
aux = J*omega;
omega_dot = inv(J)*([tau_1_ref; tau_2_ref; tau_3_ref] - cross(omega, aux));

rhs=([vel;...
      acc;...
      qdot;...
      omega_dot]);

%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{rhs}); 
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states));
%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
g = [];  % restricciones de estados del problema  de optimizacion

%%EMPY VECTOR ERRORS
he = [];
obj_he = 0;

%% EMPY VECTOR CONTROL VALUES
obj_u = 0;
u = [];

obj_quat = 0;
%% INITIAL CONDITION ERROR
st  = X(:,1); % initial state
g = [g;X(:,1)-P(1:13)]; % initial condition constraints

%% Matrix definition 
Q = 1*eye(3);
R = 1*eye(n_control);

R(1, 1) = 1/F_max;
R(2, 2) = 1/tau_1_max;
R(3, 3) = 1/tau_2_max;
R(4, 4) = 1/tau_3_max;

%% Function error in quaternions
q_error = MX.sym('q_error', 4, 1);
f_error = Function('f', {q_error}, {if_else(q_error(1) >= 0, q_error, -q_error)});
%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    st = X(:,k);  con = U(:,k);

    %% Funcion costo a minimizar 
    he = X(1:3,k)-P(13*k+1:13*k+3);
    obj_he = obj_he + (he'*Q*he);
    u = con;
    obj_u = obj_u + (u'*R*u);
    
    %% Auxiliar section Quaternions
    q_d = P(13*k+7:13*k+10);
    q = X(7:10, k);
    q_d_c = congujate_quaternion(q_d);
    
    %% get Error inQuaternions
    q_error_aux = quaternion_multiply(q, q_d_c);
    q_error_short = f_error(q_error_aux);
    [theta, axis] = quaternionToAxisAngle(q_error_short');
    %log_error = (1/2)*abs(theta(1))*axis(1:3);
    obj_quat = obj_quat +  (1-q_error_short(1)) + q_error_short(2:4)'*q_error_short(2:4);
    
    %% Actualizacion del sistema usando Euler runge kutta
    st_next = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + ts/2*k1, con); % new
    k3 = f(st + ts/2*k2, con); % new
    k4 = f(st + ts*k3, con); % new
    st_next_RK4=st +ts/6*(k1 +2*k2 +2*k3 +k4); % new 
    
    %% Restricciones del sistema se =basan en el modelo del sistema
    g = [g;st_next-st_next_RK4]; 
end

%% Cost final 

%% FINAL COST
obj = obj_he + 0.0001*obj_u + 0.01*obj_quat;

% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,13*(N+1),1);reshape(U,4*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:13*(N+1)) = 0;
args.ubg(1:13*(N+1)) = 0;

args.lbx(1:13:13*(N+1),1) = -inf; 
args.ubx(1:13:13*(N+1),1) = inf;  

args.lbx(2:13:13*(N+1),1) = -inf; 
args.ubx(2:13:13*(N+1),1) = inf;  

args.lbx(3:13:13*(N+1),1) = -inf;
args.ubx(3:13:13*(N+1),1) = inf;  

args.lbx(4:13:13*(N+1),1) = -inf; 
args.ubx(4:13:13*(N+1),1) = inf;  

args.lbx(5:13:13*(N+1),1) = -inf; 
args.ubx(5:13:13*(N+1),1) = inf;  

args.lbx(6:13:13*(N+1),1) = -inf; 
args.ubx(6:13:13*(N+1),1) = inf;  

args.lbx(7:13:13*(N+1),1) = -inf; 
args.ubx(7:13:13*(N+1),1) = inf;  

args.lbx(8:13:13*(N+1),1) = -inf; 
args.ubx(8:13:13*(N+1),1) = inf;  

args.lbx(9:13:13*(N+1),1) = -inf; 
args.ubx(9:13:13*(N+1),1) = inf;  

args.lbx(10:13:13*(N+1),1) = -inf; 
args.ubx(10:13:13*(N+1),1) = inf;  

args.lbx(11:13:13*(N+1),1) = -inf; 
args.ubx(11:13:13*(N+1),1) = inf;  

args.lbx(12:13:13*(N+1),1) = -inf; 
args.ubx(12:13:13*(N+1),1) = inf;  

args.lbx(13:13:13*(N+1),1) = -inf; 
args.ubx(13:13:13*(N+1),1) = inf;  

%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(13*(N+1)+1:4:13*(N+1)+4*N,1) = F_min;  %
args.ubx(13*(N+1)+1:4:13*(N+1)+4*N,1) = F_max;  %

args.lbx(13*(N+1)+2:4:13*(N+1)+4*N,1) = tau_1_min;  %
args.ubx(13*(N+1)+2:4:13*(N+1)+4*N,1) = tau_1_max;  % 

args.lbx(13*(N+1)+3:4:13*(N+1)+4*N,1) = tau_2_min;  %
args.ubx(13*(N+1)+3:4:13*(N+1)+4*N,1) = tau_2_max;  %

args.lbx(13*(N+1)+4:4:13*(N+1)+4*N,1) = tau_3_min;  %
args.ubx(13*(N+1)+4:4:13*(N+1)+4*N,1) = tau_3_max;  %

end