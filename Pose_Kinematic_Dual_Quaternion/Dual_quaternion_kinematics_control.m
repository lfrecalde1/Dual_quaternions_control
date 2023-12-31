%% Control Rigid Body Kinematics only
clc, clear all, close all;

%% Time definition
ts = 0.05;
t_final = 20;
t = (0:ts:t_final);

%% Initial Position dual quaternion formulation
t_0 = [0;0;0;0];
angle_0 = 0; %% 
h = zeros(8, length(t)+1);

%% The rotation is similiar a rotation vector formation
r_o = rotation_quaternion(angle_0, [0;0;1]);
h(:,1) =  pose_dual(t_0,r_o);
p(:,1) = get_traslatation_dual(h(:, 1));
r(:,1) = get_rotation_dual(h(:, 1));
[Angle_axis(:, 1)] = quat2axang(r(:, 1)');
orientacion_aux(:, 1) = (quat2eul(r(:, 1)','ZYX'))';
euler_angles(:, 1) = [orientacion_aux(3, 1);orientacion_aux(2, 1);orientacion_aux(1, 1)];

%% Linear Velocities
p1_dot = 0.0*ones(1, length(t));
p2_dot = 0.0*ones(1, length(t));
p3_dot = 0.0*ones(1, length(t));

%% Angular Velocities
w1 = 0*ones(1, length(t));
w2 = 0*ones(1, length(t));
w3 = 0.0*ones(1, length(t));

%% Vector Velocities
p_dot = [p1_dot; p2_dot; p3_dot];
w = [w1; w2; w3];

%% Ros Connection
rosshutdown
rosinit('192.168.100.15', 'NodeHost', '192.168.100.15', 'Nodename', '/Matlab_Dual_quaternion');
odomPublisher = rospublisher('/odom', 'nav_msgs/Odometry');
odomMsg = rosmessage(odomPublisher);

RefPublisher = rospublisher('/visualization_marker', 'visualization_msgs/Marker');
RefMsg = rosmessage(RefPublisher);

%% Desired Dual Quaternions
t_d = [0;0;0;0];
angle_d = pi/2;
axis_d = [0;0;1];
p_dot_d = [1.0*cos(0.3*t);...
           1.0*sin(0.3*t);...
           0.0*sin(0.1*t)];  
       
w_d = [0.0*ones(1, length(t));...
       0.0*sin(0.1*t);...
       0.0*sin(0.1*t)];
   
%% Desired Pose Definition
[h_d, p_d, r_d, xi_d] = desired_pose(t_d, angle_d, axis_d, p_dot_d, w_d, t, ts);

%% Gains
kp_pose = 1.5*[0;1;1;1];
kp_attitude = 1.5*[0;1;1;1];
kp = [kp_pose; kp_attitude];

%% Send Initial Values Ros
send_odom(odomMsg, odomPublisher, p(:, 1), r(:, 1));
send_ref_init(RefMsg, RefPublisher, p_d(:, 1), r_d(:, 1));
for k = 1:length(t)
    tic;
    %% Get Error Cuaternions
    [log_he] = log_error_control(h_d(:, k), h(:, k));
    h_d_c(:, k) = conjugate_dual(h_d(:, k));
    he(:, k) = mult_dual(h(:, k),h_d_c(:, k));
    aux_error = he(:, k);
    
    %% Control Law of the system
    control_law = -2*inner_product_dual_vector_quaternion(kp, log_he) + 1*Ad_quat(he(:, k), xi_d(:, k));
    
    %% Split Control Values
    p_dot(:, k) = control_law(6:8);
    w(:, k) = control_law(2:4);
    
    %% Get Evolution of the system based on Jacobian
    xi(:, k) = jacobian_dual_quaternion(p_dot(:, k), p(2:4, k), w(:, k));
    xi_aux(:, k) = ((ts/2)*xi(:, k));
    
    %% Integral System Based on lie Algebra
    h(:, k+1) = expm(mult_dual_matrix(xi_aux(:, k)))*h(:, k);
    p(:, k+1) = get_traslatation_dual(h(:, k+1));
    r(:, k+1) = get_rotation_dual(h(:, k+1));

    %% Angles Auxiliar variables
    orientacion_aux(:, k+1) = (quat2eul(r(:, k+1)','ZYX'))';
    euler_angles(:, k+1) = [orientacion_aux(3, k+1);orientacion_aux(2, k+1);orientacion_aux(1, k+1)];
    [Angle_axis(:, k+1)] = quat2axang(r(:, k+1)');
    
    %% Send Ros Values
    send_odom(odomMsg, odomPublisher, p(:, k+1), r(:, k+1));
    send_ref(RefMsg, RefPublisher, p_d(:, k+1), r_d(:, k+1));
    while(toc<ts)
    end
    toc;
end
rosshutdown;

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
