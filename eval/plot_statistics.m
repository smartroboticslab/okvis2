function [ output_args ] = plot_statistics( stats, h )
%PLOT_STATISTICS Plot the statistics 
%   This funciton plots mean error and standard deviations of the
%   statistics passed as argument.
% INPUT:
%   - stats: statistics
%   - h: figure object

figure(h)
hold on
grid on

mean_val = stats{1,4}{1}{1}(1);
sigma_val = stats{1,4}{1}{1}(4);
 
errorbar(1, mean_val , sigma_val);

mean_val = stats{1,4}{2}{1}(1);
sigma_val = stats{1,4}{2}{1}(4);

errorbar(2,  mean_val , sigma_val);

mean_val = stats{1,4}{3}{1}(1);
sigma_val = stats{1,4}{3}{1}(4);

errorbar(3,  mean_val , sigma_val);

end

