%% Check property 
clc, clear all, close all;


%% Initial Position dual quaternion formulation
t_0 = [0;5;5;10];
angle_0 = pi/4; %% problems in pi "I do not the answer for that...."
%% The rotation is similiar a rotation vector formation
r_o = rotation_quaternion(angle_0, [0;0;1]);
h(:,1) =  pose_dual(t_0,r_o);
p(:,1) = get_traslatation_dual(h(:, 1));
r(:,1) = get_rotation_dual(h(:, 1));

%% Linear Velocities
p1_dot = 0.1;
p2_dot = 0.8;
p3_dot = 0.3;

%% Angular Velocities
w1 = 0.2;
w2 = 0;
w3 = 0.1;

%% Vector Velocities
p_dot = [p1_dot; p2_dot; p3_dot];
w = [w1; w2; w3];

%% Xi vector 
xi(:, 1) = jacobian_dual_quaternion(p_dot(:, 1), p(2:4, 1), w(:, 1));
%% Conjugate of the two dual quaternions
conjugate_xi = conjugate_dual(xi)
conjugate_h = conjugate_dual(h)

 aux_1 = mult_dual(conjugate_xi, conjugate_h)
 aux_3 = -mult_dual(xi, conjugate_h)