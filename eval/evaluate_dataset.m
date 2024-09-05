function [ plots, statistics] = ...
    evaluate_dataset( gt_file, data, delay, name, versions )
%EVALUATE_DATASET Summary of this function goes here
%   Detailed explanation goes here
%
%
% INPUT:
%   gt_file     - the absolute path to the ground truth file.
%   data        - vector of paths to the data files.
%   length      - the length of the trajectory.
%   delay       - the delay of the gt (dataset) w.r.t. the estimate (data).
%   name        - name to be used in the plots.
%   versions - to be used in the plots.

%% Set up variables
colours = {'r', 'b'};
markers = {'o', '+'};
line_widths = [1,1];

plots = {};
statistics = cell(size(versions,2),3);
%% Open ground truth

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



p_g_W=[C{2} C{3} C{4}];

% ground truth has Hamiltonian quaternions with [qw qx qy qz]
% matlab expects Hamilton with [qw qx qy qz]
q_g=[C{5} C{6} C{7} C{8}];

T_BV = [ 0.33638, -0.01749,  0.94156,  0.06901;
         -0.02078, -0.99972, -0.01114, -0.02781;
          0.94150, -0.01582, -0.33665, -0.12395;
              0.0,      0.0,      0.0,      1.0];
% ortogonalize
[U, ~, V]=svd(T_BV(1:3,1:3));
T_BV(1:3,1:3) = U*V'; % Drop the diagonal
det(T_BV(1:3,1:3));

          
          
q_BV = rotm2quat(T_BV(1:3,1:3));


% for i=1:size(p_g_W,1)
%    temp = (T_BV*[p_g_W(i,:)';1])'; 
%    p_g_W(i,:) = temp(1:3);   
% end

p_g_W = quatrotate(q_BV,p_g_W);
q_g = quatmultiply(q_BV, q_g);



%% plotting structure and ground truth trajectory

p_g_W_intermitted=p_g_W(1:10:size(gt_timestamp),:);
gt_time=double(gt_timestamp(1:10:size(gt_timestamp))-gt_timestamp(1))/1.0e9;
p_g_W_intermitted([false;diff(gt_time)>0.15],:)=...
    p_g_W_intermitted([false;diff(gt_time)>0.15],:)*nan;

% p_g_W_intermitted = p_g_W;
% gt_time = gt_timestamp;

cg_traj_2d = figure();
plots = {plots, cg_traj_2d};
axis equal;
hold on;
set(gcf, 'name', [name ': 2D Trajectory']);

plot_gt_2d = plot(...
    p_g_W_intermitted(:,1),...
    p_g_W_intermitted(:,2),...
    '-k','LineWidth',1);



%plot z
cg_traj_z = figure();
plots = {plots, cg_traj_z};
hold on;
set(gcf, 'name', [name ': Plot Z']);

plot_gt_z = plot(...
    gt_time(1:5:size(gt_time,1)), ...
    p_g_W_intermitted(1:5:size(gt_time,1),3),...
    '-k','LineWidth',1);

hold on;

% plot 3d
cg_traj_3d = figure();
plots = {cg_traj_3d};
set(gcf, 'name', [name ': 3D Trajectory']);
axis equal;
hold on;

plot_gt_3d = plot3(...
    p_g_W_intermitted(:,1),...
    p_g_W_intermitted(:,2),...
    p_g_W_intermitted(:,3),...
    '-k','Linewidth',1);

%% process data
for k = 1:size(data,2)
    % read estimator output file - can't use csvread due to long int timestamps
    file_id = fopen(data{k});
    C = textscan(file_id, '%d64 %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %*[^\n]','delimiter', ',');
    fclose(file_id);
    
    
    % put into human-readable data structures
    timestamp=C{1};
    
    p_e_W=[C{2} C{3} C{4}];
    
    % OKVIS outputs Hamiltonian quaternions with [qx qy qz qw]
    % MATLAB expects Hamilton with [qw qx qy qz ]
    q_e=[C{8} C{5} C{6} C{7}];
    
%     put the estimate in the correct IMU frame.
%     theta_z =  pi;
%     C_z = [cos(theta_z) -sin(theta_z) 0; ...
%            sin(theta_z) cos(theta_z) 0; ...
%            0 0 1;];
%     theta_y =  -pi/2;
%     C_y = [cos(theta_y), 0, sin(theta_y); ...
%            0           , 1, 0; ...
%            -sin(theta_y), 0, cos(theta_y)];
% 
%     q_z = rotm2quat(C_z);
%     q_e = quatmultiply(q_z, q_e);
%     p_e_W = quatrotate(q_z,p_e_W);

    %Remove the estimation outside the ground truth range
    %remove the estimates before the first measure of ground truth
    idxs = timestamp < gt_timestamp(1);
    p_e_W = p_e_W(~idxs, :);
    q_e = q_e(~idxs, :);
    timestamp = timestamp(~idxs, :);
    %remove the estimates after the last measure of ground truth
    idxs = timestamp > gt_timestamp(end);
    p_e_W = p_e_W(~idxs,:);
    q_e = q_e(~idxs, :);
    timestamp = timestamp(~idxs, :);

    q_e = quatnormalize(q_e);
    q_g = quatnormalize(q_g);

    p_g_ts = resample_positions(p_e_W, timestamp,  p_g_W, gt_timestamp);
    q_g_ts = resample_quaternion(q_e, timestamp, q_g, gt_timestamp);
    p_e_ts = p_e_W;
    q_e_ts = q_e;
    
   T_ge = eye(4);
    [ p_e_ts, q_e_ts, p_g_ts, q_g_ts, T_ge] = ...   
        align_trajectories(p_e_ts, q_e_ts, p_g_ts, q_g_ts);
    

%     [ ~, ~, ~, ~, T_ge] = ...
%         align_trajectories(p_e_ts, q_e_ts, p_g_ts, q_g_ts);
 
%     [p_e_ts, q_e_ts,T_ge] = ...                               
%         trivial_alignement(p_e_ts, q_e_ts, p_g_ts, q_g_ts);

    [ ~, ~, orient_error_ez] =  ...
        compute_errors(p_e_ts,q_e_ts,p_g_ts,q_g_ts);
    
    
    pos_error = position_error(p_g_ts, p_e_ts);
    
    orient_error = orientation_error(q_g_ts, q_e_ts);
    
    
    
    % align the trajectory to plot
    p_e_W_plot = zeros(size(p_e_W));
    for i=1:size(p_e_W,1)
        p_e_W_plot(i,:) = (T_ge(1:3,1:3)*p_e_W(i,:)' + T_ge(1:3,4))';
    end
    
    
    
    % translation error statistics
    [mean_e, rms_e, med_e, std_e, min_e, max_e] = error_statistics(pos_error);
    statistics{k,1} = {[mean_e, rms_e, med_e, std_e, min_e, max_e]};
    
    % orientation error statistics
    [mean_e, rms_e, med_e, std_e, min_e, max_e] = error_statistics(orient_error);
    statistics{k,2} = {[mean_e, rms_e, med_e, std_e, min_e, max_e]};
    
    % z axis orientation statistics
    [mean_e, rms_e, med_e, std_e, min_e, max_e] = error_statistics(orient_error_ez);
    statistics{k,3} = {[mean_e, rms_e, med_e, std_e, min_e, max_e]};
    
    % plot 2D from above
    figure(cg_traj_2d)
    hold on;
    grid on;
    plot_est_2d = plot(...
        p_e_W_plot(:,1), ...
        p_e_W_plot(:,2), ...
        'Color', colours{k}, ...
        'Marker', markers{k}, ...
        'Linewidth', line_widths(k));
    
    figure(cg_traj_z)
    hold on;
    grid on;
    plot_est_z = plot(...
        double(timestamp-gt_timestamp(1))/1.0e9,...
        p_e_W_plot(:,3),...
        'Color', colours{k}, ...
        'Linewidth', line_widths(k));
    
    
    % plot 3D
    figure(cg_traj_3d)
    view(30,30);
    grid on
    hold on;
    plot_est_3d  = plot3(p_e_W_plot(:,1),p_e_W_plot(:,2),p_e_W_plot(:,3), ...
        'Color', colours{k},...
        'Marker', markers{k},...
        'Linewidth', line_widths(k));
    
    
    
    

    
%     addpath('../common');
%     for i=1:size(p_e_ts,1)
%         hold on
%     [g1x, g1y, g1z] = draw_axes(p_e_ts(i,:),q_e_ts(i,:),gcf,1);
%     [g2x, g2y, g2z] = draw_axes(p_g_ts(i,:),q_g_ts(i,:),gcf,1);
%     hold off
%     input('wait');
%     delete(g1x);
%     delete(g1y);
%     delete(g1z);
%     delete(g2x);
%     delete(g2y);
%     delete(g2z);
%     drawnow
%     
%     end
end


end

