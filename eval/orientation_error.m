function [ error ] = orientation_error( q_gt, q_e )
%ORIENTATION_ERROR Compute the difference in rotation for couples of
%quaternions
%   TBA
%   INPUT:
%       q_gt: set of ground truth quaternions, Nx4
%       q_e: set of estimated quaternions, Nx4
%   OUTPUT:
%       error: the norm of the orientation error, Nx4

%% Average rotation error Choudhary 2016


% directly using the quaternions 
error = quatmultiply(q_gt, quatinv(q_e));
error = quat2axang(error); %nX4 [A,t]
plot(error(:,4).*error(:,1:3)*180/pi);
error = rad2deg(sqrt(sum(error(:,4).*error(:,4),2)));

end

