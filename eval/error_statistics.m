function [ mean_err, rms_err, med, std_dev, min_err, max_err] = ...
    error_statistics( error)
%ERROR_STATISTICS This function computes the statistic for 
%                 the error passed as argument.
% INPUT:
%   error: vector of errors.
%
% OUTPUT:
%   mean_err: mean error.
%   rms_err: root mean square of the error.
%   med: median of the error
%   std_dev: standard deviation of the error.
%   min_err: minimum value of the error.
%   max_err: maximum value of the error.
mean_err = mean(error);

rms_err = rms(error);

med = median(error);

std_dev = std(error);

min_err = min(error);

max_err = max(error);

end

