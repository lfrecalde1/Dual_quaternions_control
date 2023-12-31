%%% uadrotor dynamics %%
%% Clean variables
clc, clear all, close all;

%% Time definition
ts = 0.01;
t_final = 5;
t = (0:ts:t_final);

%% System Parameters
m = 1;
Jxx = 2.64e-3;
Jyy = 2.64e-3;
Jzz = 4.96e-3;
g = 9.8;
L_drone = [m, Jxx, Jyy, Jzz, g];
N = 20;

%% Initial Conditions of the system

pos = [2.0; 2.0; 1.0];                  %% qd pos
vel = [0.0; 0.0; 0.0];                  %% qd vel
quat = rotation_quaternion(3.81, [0.4896; 0.2032; 0.8480]);           %% qd quat
%quat = rotation_quaternion(pi/2, [0.0; 0.0; 1]);           %% qd quat
omega = [0.0; 0.0; 0.0;];               %% qd omega

%% Quadrotor generalized vector
x = zeros(13, length(t) + 1 -N);
x(:, 1) = [pos;...
           vel;...
           quat;...
           omega];
%% Auxiliar variable rotational matrix
R = zeros(3, 3, length(t)+1 -N);
R(:, :, 1) = quat2rotm(x(7:10, 1)');
orientacion_aux(:, 1) = (quat2eul(x(7:10, 1)','ZYX'))';
euler_angles(:, 1) = [orientacion_aux(3, 1);orientacion_aux(2, 1);orientacion_aux(1, 1)];

%% Control Variables
F = 0*ones(1, length(t));
M = 0*ones(3, length(t));

%% Desired trajectory 
[hxd, hyd, hzd, hpsid, hxdp, hydp, hzdp, hpsidp] = Trajectory(3, t, 5);
psi_d = Angulo(hpsid);

%% Desired angular velocity
w_d = 0*ones(3, length(t));
w_d(3, :) = hpsidp;
quat_d(1:4, 1) =  eul2quat([psi_d(1), 0, 0], 'ZYX');

%% Get desired Quaternio
[quat_d] = desired_quaternions_values(quat_d, w_d, t, ts);
%% GENERALIZED DESIRED SIGNALS
hd = [hxd;...
      hyd;...
      hzd];

%% Optimization problem
bounded = [m*g + 10; 0; 1; -1; 1; -1; 1; -1];
[f, solver, args] = mpc_drone(bounded, N, L_drone, ts);

%% Init Horizon values
vc = zeros(N,4);
H0 = repmat(x(:, 1),1,N+1)'; 

%% Contol loop
for k = 1:length(t)-N
    %% Control Section
    tic;
    [H0, control] = NMPC(x(:, k), hd, quat_d, k, H0, vc, args, solver ,N);
    toc
    F(1, k) = control(1,1);
    M(1, k) = control(1,2);
    M(2, k) = control(1,3);
    M(3, k) = control(1,4);
    
    %% System Evolution
    x(:, k +1) = system_simulation(x(:, k), F(:, k), M(:, k), L_drone, ts);
    R(:, :, k+1) = quat2rotm(x(7:10, k+1)');
    orientacion_aux(:, k+1) = (quat2eul(x(7:10, k+1)','ZYX'))';
    euler_angles(:, k+1) = [orientacion_aux(3, k+1);orientacion_aux(2, k+1);orientacion_aux(1, k+1)];
    
    
    % Optimal Values control and states
    vc = [control(2:end,:);control(end,:)];
    H0 = [H0(2:end,:);H0(end,:)];
end

%% System Simulation
close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
Drone_Parameters(0.02);
G2=Drone_Plot_3D(x(1,1),x(2,1),x(3,1),R(:, :, 1));hold on
plot3(x(1,1),x(2,1),x(3,1),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on
plot3(hd(1,1),hd(2,1),hd(3,1),'-','Color',[50,50,5]/255,'linewidth',1.5);hold on,grid on

view(-70,35);
for k = 1:20:length(t)-N
    drawnow
    delete(G2);
    G2=Drone_Plot_3D(x(1,k),x(2,k),x(3,k),R(:, :, k));hold on
    plot3(x(1,1:k),x(2,1:k),x(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    plot3(hd(1,1:k),hd(2,1:k),hd(3,1:k),'-','Color',[50,50,50]/255,'linewidth',1.5);hold on,grid on
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end


%% Plot Position Evolution
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t(1, 1:length(hd)), hd(1,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(x)), x(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
legend({'${{h_{xd}}}$', '${{h_{x}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Position Evolution}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(t(1, 1:length(hd)), hd(2,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(x)), x(2,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${{h_{yd}}}$', '${{h_{y}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);

subplot(3,1,3)
plot(t(1, 1:length(hd)), hd(3,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(x)), x(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
legend({'${{h_{zd}}}$', '${{h_{z}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$time[s]$','Interpreter','latex','FontSize',9);

%% Plot Euler Angles of the system
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t(1, 1:length(euler_angles)), euler_angles(1,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'${{\phi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles System}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(t(1, 1:length(euler_angles)),euler_angles(2,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'${\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(t(1, 1:length(euler_angles)), euler_angles(3,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'${\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$time[s]$','Interpreter','latex','FontSize',9);

%% Quaternions Values
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t(1, 1:length(quat_d)), quat_d(1,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(x)), x(7,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on
legend({'${{q_{wd}}}$', '${{q_{w}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Attitude Evolution}$','Interpreter','latex','FontSize',9);


subplot(4,1,2)
plot(t(1, 1:length(quat_d)), quat_d(2,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(x)), x(8,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${{q_{xd}}}$', '${{q_{x}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')


subplot(4,1,3)
plot(t(1, 1:length(quat_d)), quat_d(3,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(x)), x(9,:),'-','Color',[26,115,160]/255,'linewidth',1); hold on
legend({'${{q_{yd}}}$', '${{q_{y}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')

subplot(4,1,4)
plot(t(1, 1:length(quat_d)), quat_d(4,:),'--','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
plot(t(1, 1:length(x)), x(10,:),'-','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${{q_{zd}}}$', '${{q_{z}}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
xlabel('$time[s]$','Interpreter','latex','FontSize',9);

%% Plot Control Values
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,1,1)
plot(t(1, 1:length(F)), F(1,:),'-','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$f_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Force Values}$','Interpreter','latex','FontSize',9);
ylabel('$[N]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t(1, 1:length(M)), M(1,:),'-','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_x$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Torque Values}$','Interpreter','latex','FontSize',9);
ylabel('$[N.m]$','Interpreter','latex','FontSize',9);

subplot(3,1,2)
plot(t(1, 1:length(M)), M(2,:),'-','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_y$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N.m]$','Interpreter','latex','FontSize',9);

subplot(3,1,3)
plot(t(1, 1:length(M)), M(3,:),'-','Color',[50,50,50]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_z$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N.m]$','Interpreter','latex','FontSize',9);
xlabel('$time[s]$','Interpreter','latex','FontSize',9);