function [pos_error, orient_error, orient_error_ez] =  ... 
    compute_errors(p_e, q_e, p_g, q_g)
%ERRORS_AND_ALIGN Summary of this function goes here
%   Detailed explanation goes here

%% compute translation error for each point
pos_error = p_e - p_g;
pos_error = pos_error.*pos_error;
pos_error = sqrt(sum(pos_error,2));

%% compute orientation error and orientation of z error
orient_error = zeros(size(q_e,1),1);
orient_error_ez = zeros(size(q_e,1),1);

rpy_error = zeros(size(q_e,1),3);

z = [0,0,1];
for i=1:size(orient_error,1)
    q_g_i = q_g(i,:);
    q_g_inv = quatinv(q_g_i);
    q_e_i = q_e(i,:);
    
    % q_err = q_g^-1 * q_e
    %     o_err = qplus(quatInv(q_g))*q_e;
    o_err = quatmultiply(q_g_inv,q_e_i);
    [r, p, y] = quat2angle(o_err, 'XYZ');
    rpy_error(i,1) = rad2deg(r);
    rpy_error(i,2) = rad2deg(p);
    rpy_error(i,3) = rad2deg(y);
    % convert to axis angle
    % just for clarity matlab return [axis, angle]
    o_err = quat2axang(o_err);
%     o_err = o_err(1:3)*o_err(4);
    o_err = o_err(4);
    % absolute 
    orient_error(i) = rad2deg(norm(o_err));
    z_g = quatrotate(q_g_i, z);
    z_e = quatrotate(q_e_i, z);
    orient_error_ez(i) = rad2deg(acos(min(z_g*z_e',1)));
end



