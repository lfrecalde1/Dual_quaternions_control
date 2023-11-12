%%Rigid Body Dynamics 
clc, clear all, close all;

%% Time definition
ts = 0.005;
t_final = 20;
t = (0:ts:t_final);

%% Initial Position dual quaternion formulation
t_0 = [0;0;0;00];
angle_0 = 0; 
h = zeros(8, length(t)+1);
%% The rotation is similiar a rotation vector formation
r_o = rotation_quaternion(angle_0, [0;0;1]);
h(:,1) =  pose_dual(t_0,r_o);
p(:,1) = get_traslatation_dual(h(:, 1));
r(:,1) = get_rotation_dual(h(:, 1));
[Angle_axis(:, 1)] = quat2axang(r(:, 1)');
orientacion_aux(:, 1) = (quat2eul(r(:, 1)','ZYX'))';
euler_angles(:, 1) = [orientacion_aux(3, 1);orientacion_aux(2, 1);orientacion_aux(1, 1)];

%% Vector Velocities
p_dot = 0*ones(3, length(t));
w = 0*ones(3, length(t));

%% Initial Conditions xi
xi(:, 1) = jacobian_dual_quaternion(p_dot(:, 1), p(2:4, 1), w(:, 1));
xi_aux(:, 1) = ((ts/2)*xi(:, 1));
%% Desired States of the system
t_d = [0;1;1;1];
angle_d = pi/2; %% problems in pi "I do not the answer for that...."
axis_d = [1;3;2];

%% The rotation is similiar a rotation vector formation
r_d_init = rotation_quaternion(angle_d, axis_d/norm(axis_d));
h_d(:,1) =  pose_dual(t_d,r_d_init);
p_d(:,1) = get_traslatation_dual(h_d(:, 1));
r_d(:,1) = get_rotation_dual(h_d(:, 1));
%% Create Vector of the desired Values
p_d = p_d.*ones(1, length(h));
r_d = r_d.*ones(1, length(h));

%% Gains
kp_pose = [0;0.8;0.8;0.8];
kp_attitude = [0;0.8;0.8;0.8];
kp = [kp_pose; kp_attitude];

%% Control Actions 
force = zeros(3, length(t));
force(1, :) = 0.5;
tau = zeros(3, length(t));

for k = 1:length(t)
    %% Get Error Cuaternions
    h_d_c(:, k) = conjugate_dual(h_d(:, 1));
    he(:, k) = mult_dual(h(:, k),h_d_c(:, k));
    [log_he] = log_error(he(:, k));
    
    
    %% Dynamics Section
    xi_dot = f_dynamics(p(2:4, k), p_dot(:, k), w(:, k), force(:, k), tau(:, k));
    
    %% Get Evolution of the system based on Jacobian
    xi(:, k + 1) = xi(:, k) + xi_dot*ts;
    xi_aux(:, k +1) = ((ts/2)*xi(:, k + 1));

    %% Integral System Based on lie Algebra
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
plot(euler_angles(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'${{\phi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles System}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(euler_angles(2,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'${\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(euler_angles(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'${\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

%% Plot Control velocities linear
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(p_dot(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'${{v_x}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control actions}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(p_dot(2, :),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'${v_y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(p_dot(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'$v_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$m/s$','Interpreter','latex','FontSize',9);

%% PLot Angular velocities
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(w(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'${{w_x}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control actions}$','Interpreter','latex','FontSize',9);
ylabel('$[r/s]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(w(2, :),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'${w_y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[r/s]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(w(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'$w_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$r/s$','Interpreter','latex','FontSize',9);

%% Plot Position Evolution
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(p_d(2,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(p(2,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
legend({'${{h_{xd}}}$', '${{h_{x}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Position Evolution}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(p_d(3,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(p(3,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${{h_{yd}}}$', '${{h_{y}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);

subplot(3,1,3)
plot(p_d(4,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(p(4,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
legend({'${{h_{zd}}}$', '${{h_{z}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(r_d(1,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(r(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
legend({'${{q_{wd}}}$', '${{q_{w}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Attitude Evolution}$','Interpreter','latex','FontSize',9);


subplot(4,1,2)
plot(r_d(2,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(r(2,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${{q_{xd}}}$', '${{q_{x}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')


subplot(4,1,3)
plot(r_d(3,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(r(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
legend({'${{q_{yd}}}$', '${{q_{y}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')

subplot(4,1,4)
plot(r_d(4,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(r(4,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${{q_{zd}}}$', '${{q_{z}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
