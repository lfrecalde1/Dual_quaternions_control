function [h_d, p_d, r_d, xi_d] = desired_pose(t_d, angle_d, axis_d, p_dot_d, w_d, t, ts)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%% The rotation is similiar a rotation vector formation
r_d_init = rotation_quaternion(angle_d, axis_d/norm(axis_d));
h_d(:,1) =  pose_dual(t_d,r_d_init);
p_d(:,1) = get_traslatation_dual(h_d(:, 1));
r_d(:,1) = get_rotation_dual(h_d(:, 1));

for k = 1:length(t)
    %% Get Evolution of the system based on Jacobian
    xi_d(:, k) = jacobian_dual_quaternion(p_dot_d(:, k), p_d(2:4, k), w_d(:, k));
    xi_aux_d(:, k) = ((ts/2)*xi_d(:, k));
    
    %% Integral System Based on lie Algebra
    h_d(:, k+1) = expm(mult_dual_matrix(xi_aux_d(:, k)))*h_d(:, k);
    p_d(:, k+1) = get_traslatation_dual(h_d(:, k+1));
    r_d(:, k+1) = get_rotation_dual(h_d(:, k+1)); 

end
end

