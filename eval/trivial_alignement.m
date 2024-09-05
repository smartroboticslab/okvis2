function [ p_e_aligned, q_e_aligned, T_ge ] = trivial_alignement( p_e, q_e, p_gt, q_gt )
%TRIVIAL_ALIGNEMENT Aligns the estimate of the trajectory to the ground
%truth
%   This function computes the transformation between the two trajectories
%   (estimated and ground truth) using the first position.
%   INPUT:
%       p_e: estimated position
%       q_e: estimated orientation
%       p_gt: ground truth position
%       q_qt: ground truth orientation
%   OUTPUT:
%       p_e_aligned: estimated positions aligned to the ground truth
%       q_e_aligned: estimated orientation aligned to the ground truth

index = 5;

T_Wg = eye(4);
T_Wg(1:3,1:3) = quat2rotm(q_gt(index,:));
T_Wg(1:3,4) = p_gt(index,:)';

T_We = eye(4);
T_We(1:3,1:3) = quat2rotm(q_e(index,:));
T_We(1:3,4) = p_e(index,:)';

T_WgWe = T_Wg*inv(T_We);

% q_e_gt = quatmultiply(q_gt(20,:),quatinv(q_e(20,:)));
% 
% t_e_gt = p_gt(20,:) - p_e(20,:);

q_gt_e = rotm2quat(T_WgWe(1:3,1:3));
t_gt_e = T_WgWe(1:3,4)';

p_e_aligned = quatrotate(q_gt_e,p_e) + repmat(t_gt_e,size(p_e,1),1);
q_e_aligned = quatmultiply(q_gt_e, q_e);

% T_ge = eye(4);
% T_ge(1:3,4) = t_e_gt;
% T_ge(1:3,1:3) = quat2rotm(q_e_gt);

T_ge = T_WgWe;


end

