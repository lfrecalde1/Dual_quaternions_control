%%%%%%%%%%%%%%%%%%Rigid Body Dynamics Controller Based on Dual
%%%%%%%%%%%%%%%%%%Quaternions%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clear all, close all;

%% Time definition
ts = 0.01;
t_final = 20;
t = (0:ts:t_final);

%% System Values Dyanmics
m = 50;
J_xx = 1.0;
J_yy = 0.93;
J_zz = 0.85;
parameters_system = [m, J_xx, J_yy, J_zz];

%% Inertia Matrix
J = [J_xx, 0, 0;...
     0, J_yy, 0;...
     0, 0, J_zz];
 
%% Initial Position dual quaternion formulation
t_0 = [0;2;2;1];
angle_0 = 3.81; 
h = zeros(8, length(t)+1);

%% Initial Conditions Based on Dual Quaternions
r_o = rotation_quaternion(angle_0, [0.4896; 0.2032; 0.8480]);
h(:,1) =  pose_dual(t_0,r_o);
p(:,1) = get_traslatation_dual(h(:, 1));
r(:,1) = get_rotation_dual(h(:, 1));
[Angle_axis(:, 1)] = quat2axang(r(:, 1)');
orientacion_aux(:, 1) = (quat2eul(r(:, 1)','ZYX'))';
euler_angles(:, 1) = [orientacion_aux(3, 1);orientacion_aux(2, 1);orientacion_aux(1, 1)];

%% Vector Velocities linear and Angular
p_dot = 0*ones(3, length(t));
w = 0*ones(3, length(t));

%% Initial Conditions xi
xi(:, 1) = jacobian_dual_quaternion(p_dot(:, 1), p(2:4, 1), w(:, 1));
xi_aux(:, 1) = ((ts/2)*xi(:, 1));

%% Desired States of the system
t_d = [0;0;0;0];
angle_d = 0; 
axis_d = [0;0;1];

%% Desired Dual Quaternion
r_d_init = rotation_quaternion(angle_d, axis_d/norm(axis_d));
h_d(:,1) =  pose_dual(t_d,r_d_init);
p_d(:,1) = get_traslatation_dual(h_d(:, 1));
r_d(:,1) = get_rotation_dual(h_d(:, 1));

%% Create Vector of the desired Values
p_d = p_d.*ones(1, length(t));
r_d = r_d.*ones(1, length(t));

%% Desired Linear and Angular Velocities
p_dot_d = [0; 0; 0];
w_d = [0; 0; 0];
xi_d(:, 1) = jacobian_dual_quaternion(p_dot_d(:, 1), p_d(2:4, 1), w_d(:, 1));

%% Control Actions 
force = zeros(3, length(t));
tau = zeros(3, length(t));

%% Control gains
kp = 1;
kd = 2;
for k = 1:length(t)-1
    %% Get Error Cuaternions
    [U] = Dynamic_control_dual_quaternion(h_d(:, 1), h(:, k), xi_d(:, 1), xi(:, k), p(2:4, k), p_dot(:, k), w(:, k), parameters_system, kp, kd);
    
    %% Split Control Actions
    force(:, k) = m*(U(6:8)- cross(U(2:4), p(2:4, k)));
    tau(:, k) = J*U(2:4);
    
    %% Dynamics Section
    xi_dot = f_dynamics(p(2:4, k), p_dot(:, k), w(:, k), force(:, k), tau(:, k), parameters_system);
    
    %% Integral System Based on lie Algebra
    xi(:, k + 1) = xi(:, k) + xi_dot*ts;
    xi_aux(:, k +1) = ((ts/2)*xi(:, k + 1));
    h(:, k+1) = expm(mult_dual_matrix(xi_aux(:, k+1)))*h(:, k);
    p(:, k+1) = get_traslatation_dual(h(:, k+1));
    r(:, k+1) = get_rotation_dual(h(:, k+1));
    w(:, k+1) = xi(2:4, k + 1);
    p_dot(:, k+1) = xi(6:8, k + 1) - cross(p(2:4, k+1), w(:, k+1));
    
    %% Angles Auxiliar variables
    orientacion_aux(:, k+1) = (quat2eul(r(:, k+1)','ZYX'))';
    euler_angles(:, k+1) = [orientacion_aux(3, k+1);orientacion_aux(2, k+1);orientacion_aux(1, k+1)];
    [Angle_axis(:, k+1)] = quat2axang(r(:, k+1)');
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

subplot(1,1,1)
plot3(p(2,:), p(3,:), p(4,:), '-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot3(p_d(2,:), p_d(3,:), p_d(4,:), '*','Color',[100,100,100]/255,'linewidth',1); hold on
grid on;
legend({'${p}$',},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

%% Plot Euler Angles of the system
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t, euler_angles(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'${{\phi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles System}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(t,euler_angles(2,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'${\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(t, euler_angles(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'${\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$time[s]$','Interpreter','latex','FontSize',9);

%% Plot Control velocities linear
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t, p_dot(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'${{v_x}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Linear Velocities}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(t, p_dot(2, :),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'${v_y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(t, p_dot(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'$v_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$m/s$','Interpreter','latex','FontSize',9);
xlabel('$time[s]$','Interpreter','latex','FontSize',9);

%% PLot Angular velocities
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t, w(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'${{w_x}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angular Velocities}$','Interpreter','latex','FontSize',9);
ylabel('$[r/s]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(t, w(2, :),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'${w_y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[r/s]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(t, w(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'$w_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$r/s$','Interpreter','latex','FontSize',9);
xlabel('$time[s]$','Interpreter','latex','FontSize',9);

%% Plot Position Evolution
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t, p_d(2,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t, p(2,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
legend({'${{h_{xd}}}$', '${{h_{x}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Position Evolution}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(t, p_d(3,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t, p(3,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${{h_{yd}}}$', '${{h_{y}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);

subplot(3,1,3)
plot(t, p_d(4,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t, p(4,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
legend({'${{h_{zd}}}$', '${{h_{z}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$time[s]$','Interpreter','latex','FontSize',9);
 
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t, r_d(1,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t, r(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
legend({'${{q_{wd}}}$', '${{q_{w}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Attitude Evolution}$','Interpreter','latex','FontSize',9);


subplot(4,1,2)
plot(t, r_d(2,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t, r(2,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${{q_{xd}}}$', '${{q_{x}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')


subplot(4,1,3)
plot(t, r_d(3,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t, r(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
legend({'${{q_{yd}}}$', '${{q_{y}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')

subplot(4,1,4)
plot(t, r_d(4,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t, r(4,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${{q_{zd}}}$', '${{q_{z}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
xlabel('$time[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t, force(1,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$f_x$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Force Values}$','Interpreter','latex','FontSize',9);
ylabel('$[N]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(t, force(2,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$f_y$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N]$','Interpreter','latex','FontSize',9);

subplot(3,1,3)
plot(t, force(3,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$f_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N]$','Interpreter','latex','FontSize',9);
xlabel('$time[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t, tau(1,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_x$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Torque Values}$','Interpreter','latex','FontSize',9);
ylabel('$[N.m]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(t, tau(2,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_y$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N.m]$','Interpreter','latex','FontSize',9);

subplot(3,1,3)
plot(t, tau(3,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N.m]$','Interpreter','latex','FontSize',9);
xlabel('$time[s]$','Interpreter','latex','FontSize',9);
