%% VIO Evaluation
% written by lestefan 07/2013

clear variables global
close all


%% configurable paramenters - edit

% dataset names
rpg = 0;
dataset = 'tumvi';
dataset = 'euroc';
datapath = '/Users/sleutene/Documents/datasets';
datapath = '/Users/leuteneg/Documents/datasets';
%datapath = '/srv/user/leuteneg/datasets';

%dataset_names = {'indoor_forward_7_snapdragon_with_gt'};
%dataset_names = {'dataset-outdoors1_512_16'};
%dataset_names = {'dataset-corridor1_512_16'};
%dataset_names = {'dataset-magistrale4_512_16'};
%dataset_names = {'dataset-room1_512_16'};
%dataset_names = {'dataset-slides1_512_16'};
%dataset_names = {'MH_01_easy'};
%dataset_names = {'MH_02_easy'};
%dataset_names = {'MH_03_medium'};
%dataset_names = {'MH_04_difficult'};
dataset_names = {'MH_05_difficult'};
%dataset_names = {'V1_01_easy'};
%dataset_names = {'V1_02_medium'};
%dataset_names = {'V1_03_difficult'};
%dataset_names = {'V2_01_easy'};
%dataset_names = {'V2_02_medium'};
%dataset_names = {'V2_03_difficult'};
%dataset_names = {'vicon_room_01_01'};

% estimator versions
versions = {'okvis2-vio', 'okvis2-slam', 'okvis2-slam-final'};
versions = {'okvis2-slam', 'okvis2-slam-final'};
%versions = {'okvis2-vio-final','okvis2-slam-final'};
colours = {[0,0,1], [1,0,1], [1,0.5,0.5]};

% discretisation
Delta_s = 10.0; % [m]


%% processing
if dataset == 'tumvi'
    gtname = 'mocap0'; 
else 
    gtname = 'state_groundtruth_estimate0';
end

% folders
%dataset_folder = '/Users/sleutene/Documents/datasets/fpv';
dataset_folder = [datapath '/' dataset '/'];
groundtruth_folder = [datapath '/' dataset '/'];

% groundtruth delay w.r.t. the estimator output
delay_gt_estimates = [0.0, 0.0, 0.0, 0.0, 0.0];


%% evaluate dataset
statistics = cell(size(dataset_names,2), 4);

for i=1:size(dataset_names,2)
    name = dataset_names{i};
    fprintf('Evaluating dataset %s with estimator versions ', name);
    for k=1:size(versions,2)
        fprintf('%s ', versions{k});
    end
    fprintf('\n');
    
    if rpg
        ground_truth = [groundtruth_folder '/' name '/mav0/' 'groundtruth.txt'];
    else
        ground_truth = [groundtruth_folder '/' name '/mav0/' gtname '/data.csv'];
    end
    fprintf('\tGround truth file: %s\n', ground_truth);
    
    estimate_files = cell(size(versions));
    for k=1:size(versions,2)
        estimate_files{k} = [dataset_folder '/' name '/mav0/' versions{k} '_trajectory.csv'];
        fprintf('\tFile with estimated states: %s\n', estimate_files{k});
    end
    
    %generate plots and statistics
    [pos_buckets, rot_buckets] = evaluate_dataset_relative(...
        ground_truth, ...
        estimate_files, ...
        delay_gt_estimates(k),...
        versions, Delta_s, rpg);
    
    figure;
    title(dataset_names{i})
    subplot(3,1,1)
    ylabel('Position error [m]')
    hold on;
    subplot(3,1,2)
    ylabel('Tilt error [deg]')
    hold on;
    subplot(3,1,3)
    ylabel('Azimuth error [deg]')
    xlabel('Distance travelled [m]')
    hold on;
    for k=1:length(pos_buckets)
        num_buckets = length(pos_buckets{k});
        X = zeros(num_buckets,1);
        Y = zeros(num_buckets,3);
        E_l = zeros(num_buckets,3);
        E_h = zeros(num_buckets,3);
        for s=1:length(pos_buckets{k})
            abs_pos_err = vecnorm(pos_buckets{k}{s});
            m = median(abs_pos_err);
            ep_low  = prctile(abs_pos_err,05);
            ep_high = prctile(abs_pos_err,95);
            X(s) = Delta_s/2+(s-1)*Delta_s+k;
            Y(s,1) = m;
            E_l(s,1) = ep_low-m;
            E_h(s,1) = m-ep_high;
            %subplot(3,1,1)
            %errorbar(Delta_s/2+(s-1)*Delta_s+k, m, ep_low - m, m - ep_high,...
            %    'o', 'MarkerSize', 5, 'LineWidth', 0.5, 'Color', colours{k});
            
            rp_err = 180/pi*vecnorm(rot_buckets{k}{s}(1:2,:));
            m = median(rp_err);
            ep_low  = prctile(rp_err,05);
            ep_high = prctile(rp_err,95);
            Y(s,2) = m;
            E_l(s,2) = ep_low-m;
            E_h(s,2) = m-ep_high;
            %subplot(3,1,2)
            %errorbar(Delta_s/2+(s-1)*Delta_s+k, m, ep_low - m, m - ep_high,...
            %    'o', 'MarkerSize', 5, 'LineWidth', 0.5, 'Color', colours{k});
            
            y_err = 180/pi*abs(rot_buckets{k}{s}(3,:));
            m = median(y_err);
            ep_low  = prctile(y_err,05);
            ep_high = prctile(y_err,95);
            Y(s,3) = m;
            E_l(s,3) = ep_low-m;
            E_h(s,3) = m-ep_high;
            %subplot(3,1,3)
            %errorbar(Delta_s/2+(s-1)*Delta_s+k, m, ep_low - m, m - ep_high,...
            %    'o', 'MarkerSize', 5, 'LineWidth', 0.5, 'Color', colours{k});
        end
        subplot(3,1,1)
        errorbar(X, Y(:,1), E_l(:,1), E_h(:,1),...
            'o', 'MarkerSize', 5, 'LineWidth', 0.5, 'Color', colours{k});
        subplot(3,1,2)
        errorbar(X, Y(:,2), E_l(:,2), E_h(:,2),...
            'o', 'MarkerSize', 5, 'LineWidth', 0.5, 'Color', colours{k});
        subplot(3,1,3)
        errorbar(X, Y(:,3), E_l(:,3), E_h(:,3),...
            'o', 'MarkerSize', 5, 'LineWidth', 0.5, 'Color', colours{k});
    end
    legend(versions)
    
    % absolute evaluation
    [p_g_ts_all, q_g_ts_all, p_e_ts_all, q_e_ts_all, p_eg_ts_all, q_eg_ts_all] = ...
        evaluate_dataset_absolute( ground_truth, ...
        estimate_files, ...
        delay_gt_estimates(k),...
        versions, rpg);
    
    figure(2)
    %plot3(p_g_ts_all{1}(:,1), p_g_ts_all{1}(:,2), p_g_ts_all{1}(:,3),'-k')
    plot(p_g_ts_all{1}(:,1), p_g_ts_all{1}(:,2),':', ...
        'LineWidth', 2, 'Color', [0.8,0.8,0.8])
    axis equal;
    hold on;
    for k=1:length(versions)
        %plot3(p_e_ts_all{k}(:,1), p_e_ts_all{k}(:,2), ...
        %    p_e_ts_all{k}(:,3), 'Color', colours{k})
        plot(p_eg_ts_all{k}(:,1), p_eg_ts_all{k}(:,2), 'Color', colours{k})
        
        % display RMSE of ATE
        err = p_eg_ts_all{k}-p_g_ts_all{k};
        sq = err.*err;
        sq_ate = (sum(sq,2));
        rmse = sqrt(mean(sq_ate));
        %ate = sqrt(sum(sq,2));
        %mate = mean(ate)
        versions_rmse{k+1} = strcat(versions{k},", ", string(rmse), " m");        
    end
end

figure(2)
versions_rmse{1} = "Ground truth";
legend(versions_rmse);

% destination file name
% mkdir(result_folder);
% result_file = [result_folder '/' 'results.tex'];

% write_statistics_table(result_file, statistics);
