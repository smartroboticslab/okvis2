%% set up
clear 
close all
clc

addpath('../slam_workbench');


%% parameters 

radious = 1;
rot_e = angle2quat(0,0,pi/6); % ZYX
%  rot_e = [1,0,0,0];



%% create two trajectories

% ground truth
[p_gt, q_gt] = generate_trajectory([0,0,0], [1,0,0,0]);


timestamps = 1:1:size(p_gt,1);

%subsample
p_gt = p_gt(1:2:end,:);
q_gt = q_gt(1:2:end,:);
timestamps_gt = timestamps(1:2:end);


% estimate 
[p_e, q_e] = generate_trajectory([0,0,0], rot_e);

p_e = [p_e(1,:); p_e(2:2:end,:)];
q_e = [q_e(1,:); q_e(2:2:end,:)];
timestamps_e  = [timestamps(1), timestamps(2:2:end)];

%% plot the two before aligning
raw = figure ;
hold on
grid on
axis equal
axis([-3,3,-3,3,-3,3])
xlabel('X');
ylabel('Y');
zlabel('Z');

plot3(p_gt(:,1), p_gt(:,2), p_gt(:,3), 'k');
plot3(p_e(:,1), p_e(:,2), p_e(:,3), 'r');
draw_axes(p_gt,q_gt,raw, 1);
draw_axes(p_e,q_e,raw,1);



%% risample

% [p_e_ts, q_e_ts, p_gt_ts, q_gt_ts] = ... 
%    resample_trajectories(p_e, q_e, timestamps_e', p_gt, q_gt,timestamps_gt',0);
% p_e = p_e_ts.Data;
% q_e = q_e_ts.Data;
% p_gt = p_gt_ts.Data;
% q_gt = q_gt_ts.Data;

p_gt = resample_positions(p_e, timestamps_e',  p_gt, timestamps_gt');
q_gt = resample_quaternion(q_e, timestamps_e', q_gt, timestamps_gt');

p_e = p_e(1:end-1,:);
q_e = q_e(1:end-1,:);
p_gt = p_gt(1:end-1,:);
q_gt = q_gt(1:end-1,:);

%% align the two trajectories
[p_e, q_e, p_gt, q_gt] = align_trajectories(p_e,q_e,p_gt,q_gt);
%[p_e,q_e] = trivial_alignement(p_e, q_e, p_gt, q_gt);

aligned = figure ;
hold on
grid on
axis equal

xlabel('X');
ylabel('Y');
zlabel('Z');

plot3(p_gt(:,1), p_gt(:,2), p_gt(:,3), 'k');
plot3(p_e(:,1), p_e(:,2), p_e(:,3), 'r');
% draw_axes(p_gt(end-1:end,:),q_gt(end-1:end,:),aligned, 1);
% draw_axes(p_e(end-1:end,:),q_e(end-1:end,:),aligned,1);
draw_axes(p_gt, q_gt,aligned, 1);
draw_axes(p_e , q_e ,aligned,1);

%% compute errors
[p_err, q_err, z_err] = compute_errors(p_e, q_e, p_gt, q_gt);


%% Compute statistics 
stats = cell(1,3);

%pose 
[mean_p_e, rms_p_e, med_p_e, std_p_e, min_p_e, max_p_e] = ...
    error_statistics(p_err);

stats{1,1} = {[mean_p_e, rms_p_e, med_p_e, std_p_e, min_p_e, max_p_e]};

%orientation 
[mean_q_e, rms_q_e, med_q_e, std_q_e, min_q_e, max_q_e] = ...
    error_statistics(q_err);

stats{1,2} = {[mean_q_e, rms_q_e, med_q_e, std_q_e, min_q_e, max_q_e]};

%z-axis
[mean_z_e, rms_z_e, med_z_e, std_z_e, min_z_e, max_z_e] = ...
    error_statistics(z_err);

stats{1,3} = {[mean_z_e, rms_z_e, med_z_e, std_z_e, min_z_e, max_z_e]};


statistics{1,1} = 'Test';
statistics{1,2} = '';
statistics{1,3} = {'ATE', 'Orientation Error', 'Z-Axis Orientation Error'};
statistics{1,4} = stats;


%% plot statistics 

s_p = figure;
hold on
grid on

plot_statistics(statistics, s_p);

title('stats plot');



