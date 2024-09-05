%% set up
clear
close all
clc
%% parameters
delay = 0;

% the files are:
% 1) state_groundtruth_estimate0/data.csv
% 2) the log of the last full bundle adjustment estimate for all the keyframes
%    of OKVIS with loop closure
% dataset: euroc dataset vicon room 1 easy
%gt_file = 'data/vicon_room_1_easy/ground_truth.csv';
gt_file = '/home/sleutene/datasets/euroc/V2_01_easy/state_groundtruth_estimate0/data.csv';
%okvis_file = 'data/vicon_room_1_easy/okvis_estimate.csv';
okvis_file = '/home/sleutene/datasets/euroc/V2_01_easy/okvis2_final_trajectory.csv';
%% read ground truth
file_id = fopen(gt_file);
%     '%d64 %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', ...

C = textscan(file_id, ...
    '%d64 %f %f %f %f %f %f %f %*[^\n]', ...
    'delimiter', ',','HeaderLines',1);
fclose(file_id);

% put into human-readable data structures
gt_timestamp=C{1};
%adjust the timestamps
gt_timestamp = gt_timestamp - delay*1e9;

p_g=[C{2} C{3} C{4}];

% ground truth has Hamiltonian quaternions with [qw qx qy qz]
% matlab expects Hamilton with [qw qx qy qz]
q_g=[C{5} C{6} C{7} C{8}];



%% read the estimate
% read estimator output file - can't use csvread due to long int timestamps
file_id = fopen(okvis_file);
C = textscan(file_id, '%d64 %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %*[^\n]','delimiter', ',','HeaderLines',1);
fclose(file_id);


% put into human-readable data structures
timestamp=C{1};

p_e=[C{2} C{3} C{4}];

% OKVIS outputs Hamiltonian quaternions with [qx qy qz qw]
% MATLAB expects Hamilton with [qw qx qy qz ]
q_e=[C{8} C{5} C{6} C{7}];

%% data processing

%Remove the estimation outside the ground truth range
%remove the estimates before the first measure of ground truth
idxs = timestamp < gt_timestamp(1);
p_e = p_e(~idxs, :);
q_e = q_e(~idxs, :);
timestamp = timestamp(~idxs, :);
%remove the estimates after the last measure of ground truth
idxs = timestamp > gt_timestamp(end);
p_e = p_e(~idxs,:);
q_e = q_e(~idxs, :);
timestamp = timestamp(~idxs, :);

q_e = quatnormalize(q_e);
q_g = quatnormalize(q_g);

p_g = resample_positions(p_e, timestamp,  p_g, gt_timestamp);
q_g = resample_quaternion(q_e, timestamp, q_g, gt_timestamp);

%plot(p_g(:,1), p_g(:,2), '-k');
%hold on;
%plot(p_e(:,1), p_e(:,2), '-r');

% align the trajectories using Horn's method
[ p_e, q_e, p_g, q_g, T_ge] = ...
    align_trajectories(p_e, q_e, p_g, q_g);

%% compute the errors

[ ~, ~, orient_error_ez] =  ...
    compute_errors(p_e,q_e,p_g,q_g);


pos_error = position_error(p_g, p_e);

orient_error = orientation_error(q_g, q_e);

%% compute the statistics
% translation error statistics
[mean_e, rms_e, med_e, std_e, min_e, max_e] = error_statistics(pos_error);
pos_error_statistics = {[mean_e, rms_e, med_e, std_e, min_e, max_e]};

% orientation error statistics
[mean_e, rms_e, med_e, std_e, min_e, max_e] = error_statistics(orient_error);
orient_error_statistics = {[mean_e, rms_e, med_e, std_e, min_e, max_e]};

fprintf('the mean absolute trajectory error is:  %f\n', pos_error_statistics{1}(1));
fprintf('the mean orientation error is: %f\n', orient_error_statistics{1}(1));

%% plot statistic over trajectory segments

number_segments = 10;
segment_size = floor(size(pos_error,1)/10);
err = figure('Name','Error over distance travelled','NumberTitle','off');

%compute distance travelled
ds=(sqrt(sum(diff(p_g).*diff(p_g),2))); % distance increments
distance_travelled=[0;cumsum(ds)];


for i=1:number_segments
    start_idx = (i-1)*segment_size + 1;
    end_idx = i*segment_size;
    if end_idx > size(pos_error,1)
        end_idx = size(pos_error,1);
    end

    
    figure(err)
    subplot(2,1,1)
    title('ATE over distance travelled [m]');
    hold on
    mean_ep = error_statistics(pos_error(start_idx:end_idx));
    ep_low  = prctile(pos_error(start_idx:end_idx),05);
    ep_high = prctile(pos_error(start_idx:end_idx),95);
    errorbar(distance_travelled(end_idx), mean_ep, ep_low - mean_ep, mean_ep - ep_high,'x', 'MarkerSize', 10, 'LineWidth', 0.5, 'Color', 'r');
    
    subplot(2,1,2)
    hold on
    title('Absolute Orientation Error Over Distance Travelled [deg]');
    mean_eo = error_statistics(orient_error(start_idx:end_idx));
    eo_low  = prctile(orient_error(start_idx:end_idx),05);
    eo_high = prctile(orient_error(start_idx:end_idx),95);
    errorbar(distance_travelled(end_idx), mean_eo, eo_low - mean_eo, mean_eo - eo_high,'x', 'MarkerSize', 10, 'LineWidth', 0.5, 'Color', 'b');
    
end
