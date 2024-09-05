function [p_g_ts_all, q_g_ts_all, p_e_ts_all, q_e_ts_all, p_eg_ts_all, q_eg_ts_all] = ...
    evaluate_dataset_absolute( gt_file, data, delay, versions, rpg )
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
if rpg
    C = textscan(file_id, ...
        '%d %f %f %f %f %f %f %f %f %*[^\n]', ...
        'delimiter', ' ','HeaderLines',1);
    C = {C{2},C{3},C{4},C{5},C{9},C{6},C{7},C{8}}; % remove seq and q:xyzw

    % put into human-readable data structures
    gt_timestamp=int64(C{1}*1e6)*1000; % microseconds only, hack...
else
    C = textscan(file_id, ...
        '%d64 %f %f %f %f %f %f %f %*[^\n]', ...
        'delimiter', ',','HeaderLines',1);

    % put into human-readable data structures
    gt_timestamp=C{1};
end
fclose(file_id);

%adjust the timestamps
gt_timestamp = gt_timestamp - delay*1e9;



p_g_W=[C{2} C{3} C{4}];

% ground truth has Hamiltonian quaternions with [qw qx qy qz]
% matlab expects Hamilton with [qw qx qy qz]
q_g=[C{5} C{6} C{7} C{8}];


%% process data

pos_mean = zeros(size(data,2),1);
ori_mean = zeros(size(data,2),1);


for k = 1:size(data,2)
    % read estimator output file - can't use csvread due to long int timestamps
    file_id = fopen(data{k});
    C = textscan(file_id, ...
        '%d64 %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %*[^\n]', ...
        'delimiter', ',','HeaderLines',1);
    fclose(file_id);

    % put into human-readable data structures
    timestamp=C{1};
    
    p_e_W=[C{2} C{3} C{4}];
    
%       figure(3)
%       plot(p_e_W(:,1), p_e_W(:,2))
%       hold on;
%       axis equal;
%       figure(4)
%       plot(diff(p_e_W(:,1)))
%       hold on;
% 
%     
%    % hack plot 
%     kfs = C{18};
%     if ~isnan(kfs)
%         figure(3)
%         plot(p_e_W(:,1), p_e_W(:,2), 'r')
%         hold on;
%         axis equal;
%         for i=1:length(kfs)
%             if kfs(i)==0
%                 plot(p_e_W(i,1), p_e_W(i,2), 'xb');
%             else
%                 plot([p_e_W(i,1); p_e_W(kfs(i),1)], [p_e_W(i,2); p_e_W(kfs(i),2)], 'y')
%             end
%         end
%         %figure(4)
%         %plot(p_e_W(:,3), 'r')
%     end
%     
    
    % OKVIS outputs Hamiltonian quaternions with [qx qy qz qw]
    % MATLAB expects Hamilton with [qw qx qy qz ]
    q_e=[C{8} C{5} C{6} C{7}];
    
    %Remove the estimation outside the ground truth range
    len = ceil(double(timestamp(length(timestamp))-timestamp(1))/0.05e9)+1;
    isValid = zeros(1,len);
    for j = 1:length(gt_timestamp)
        idx=round(double(gt_timestamp(j)-timestamp(1))/0.05e9)+1;
        if(idx>=1 && idx<=len)
          isValid(idx) = 1;
        end
    end
    idxs = logical(size(timestamp));
    for i = 1:length(timestamp)
       idx = floor(double(timestamp(i)-timestamp(1))/0.05e9)+1;
       if(idx>length(isValid))
          disp 'wtf' 
       end
       if isValid(idx) && timestamp(i)>gt_timestamp(1) && ...
               timestamp(i)<gt_timestamp(length(gt_timestamp))
          idxs(i) = 1; 
       else
          idxs(i) = 0;
       end
    end
    p_e_W = p_e_W(idxs,:);
    q_e = q_e(idxs, :);
    timestamp = timestamp(idxs, :);
    
    q_e = quatnormalize(q_e);
    q_g = quatnormalize(q_g);

    p_g_ts = resample_positions(p_e_W, timestamp,  p_g_W, gt_timestamp);
    q_g_ts = resample_quaternion(q_e, timestamp, q_g, gt_timestamp);
    p_e_ts = p_e_W;
    q_e_ts = q_e;
    
    p_g_ts_all{k} = p_g_ts;
    q_g_ts_all{k} = q_g_ts;
    p_e_ts_all{k} = p_e_ts;
    q_e_ts_all{k} = q_e_ts;
    
    % compute relative poses for the ground truth
    
    % create a vector of transformations
    T_gt = zeros(4,4,size(p_g_ts,1));
    T_gt(4,4,:) = 1;
    T_gt(1:3,1:3,:) = quat2rotm(q_g_ts);
    T_gt(1:3,4,:) = p_g_ts';
    
    
    % inverse transformations
    T_gt_inv = zeros(size(T_gt));
    for i=1:size(T_gt,3)
       T_gt_inv(:,:,i) = inv(T_gt(:,:,i));
    end
    
    
    % estimates
    T_e = zeros(4,4,size(p_e_ts,1));
    T_e(4,4,:) = 1;
    T_e(1:3,1:3,:) = quat2rotm(q_e_ts);
    T_e(1:3,4,:) = p_e_ts';

    T_e_inv = zeros(size(T_e));
    for i = 1:size(T_e,3)
       T_e_inv(:,:,i) = inv(T_e(:,:,i));
    end
    
    %% align
    [ p_e_n,q_e_n,p_g_n,q_g_n, T_ge] = align_trajectories(p_e_ts, q_e_ts, p_g_ts, q_g_ts); 
    p_eg_ts_all{k} = p_e_n;
    q_eg_ts_all{k} = q_e_n;

end