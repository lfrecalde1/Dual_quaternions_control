function [H0, control] = NMPC(h, hd, q_d, k, H0, vc, args, solver ,N)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
H = [h];
args.p(1:13) = H;

for i = 1:N
    args.p(13*i+1:13*i+13)=[hd(1,k+i), hd(2,k+i), hd(3,k+i),0,0,0, q_d(1, k+i),q_d(2, k+i), q_d(3, k+i), q_d(4, k+i),0,0,0];
end

args.x0 = [reshape(H0',13*(N+1),1);reshape(vc',size(vc,2)*N,1)]; % initial value of the optimization variables

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

control = reshape(full(sol.x(13*(N+1)+1:end))',4,N)';
H0 = reshape(full(sol.x(1:13*(N+1)))',13,N+1)';
end
