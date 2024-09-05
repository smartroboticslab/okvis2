function [ error ] = position_error( p_gt, p_e )
%POSITION_ERROR Compute the position error for two given trajectories
%   For the positions the error is given simply by the subtraction between
%   the two vectors of functions.
%   NOTE: do not forget to align the trajectories!
%   INPUT:
%       p_gt: ground truth position trajectory, Nx3
%       p_e:  estimated position trajectory, Nx3
%   OUTPUT:
%       error: Norm of the error for each pose, Nx1


error =  p_gt - p_e;
error = sqrt(sum(error.*error,2));


end

