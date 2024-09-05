%% VIO Evaluation
% written by lestefan 07/2013

clear variables global
close all
clc


%% configurable paramenters - edit

% dataset names
rpg = 0;
dataset_names = ...
    {'MH_01_easy', 'MH_02_easy', 'MH_03_medium', ...
    'MH_04_difficult', 'MH_05_difficult', ...
    'V1_01_easy', 'V1_02_medium', 'V1_03_difficult', ...
    'V2_01_easy', 'V2_02_medium', 'V2_03_difficult'};

dataset_names = ...
      {'dataset-corridor5_512_16','dataset-corridor2_512_16',...
       'dataset-corridor3_512_16','dataset-corridor4_512_16',...
      'dataset-corridor5_512_16'};
dataset_names = ...
      {'dataset-magistrale1_512_16','dataset-magistrale2_512_16',...
      'dataset-magistrale3_512_16',...
      'dataset-magistrale4_512_16',...
      'dataset-magistrale5_512_16','dataset-magistrale6_512_16'};
dataset_names = ...
       {'dataset-outdoors1_512_16','dataset-outdoors2_512_16',...
       'dataset-outdoors3_512_16','dataset-outdoors4_512_16',...
       'dataset-outdoors5_512_16','dataset-outdoors6_512_16',...
       'dataset-outdoors7_512_16','dataset-outdoors8_512_16'};
% dataset_names = ...
%       {'dataset-room1_512_16','dataset-room2_512_16',...
%       'dataset-room3_512_16','dataset-room4_512_16',...
%       'dataset-room5_512_16','dataset-room6_512_16'};
% dataset_names = ...
%       {'dataset-slides1_512_16','dataset-slides2_512_16',...
%       'dataset-slides3_512_16'};
%dataset_names = ...
%    {'dataset-slides3_512_16'};

%dataset_names = ...
%    {'dataset-outdoors4_512_16'};
%dataset_names = {'MH_02_easy'};
% 
% dataset_names = ...
%     {'vicon_room_01_01'};


% folders
dataset_folder = '/srv/user/leuteneg/datasets/euroc/2022-04-17';
dataset_folder = '/srv/user/leuteneg/datasets/tumvi/2022-04-21';
%dataset_folder = ... %'/Users/sleutene/Dropbox/okvis2_eval/euroc/allresults/';
%    '/srv/user/leuteneg/datasets/tumvi';
groundtruth_folder = ... %'/Users/sleutene/Documents/datasets/euroc/';
    '/srv/user/leuteneg/datasets/euroc';
groundtruth_folder = ... %'/Users/sleutene/Documents/datasets/euroc/';
    '/srv/user/leuteneg/datasets/tumvi';

gtname = 'state_groundtruth_estimate0';
gtname = 'mocap0'; 

% estimator versions
versions = {'okvis', 'okvis2-vio', 'okvis2-slam', 'okvis2-slam-final'};
%versions = {'okvis', 'okvis2-vio', 'okvis2-vio-calib'};
colours = {[0.5,0.5,0.5], [0,0,1], [1,0,1], [1,0.5,0.5]};
%numbers = {'01', '02', '03', '04', '05'};
numbers = {'01', '02', '03'};

% groundtruth delay w.r.t. the estimator output
delay_gt_estimates = [0.0, 0.0, 0.0, 0.0, 0.0];

% discretisation
Delta_s = 10.0; % [m]

%% evaluate dataset
statistics = cell(size(dataset_names,2), 4);

for i=1:size(dataset_names,2)
    name = dataset_names{i};
    %fprintf('Evaluating dataset %s with estimator versions ', name);
    for k=1:size(versions,2)
        fprintf('%s ', versions{k});
    end
    fprintf('\n');
    
    ground_truth = [groundtruth_folder '/' name '/mav0/' gtname '/data.csv'];
    %fprintf('Ground truth file: %s\n', ground_truth);
    
    estimate_files = cell(size(versions));
    disp(name)
    rmses = zeros(size(versions,2),size(numbers,2));
    for j=1:size(numbers,2)
    
        for k=1:size(versions,2)
            estimate_files{k} = [dataset_folder '/' name '/mav0/' numbers{j} '/' versions{k} '_trajectory.csv'];
            %fprintf('\tFile with estimated states: %s\n', estimate_files{k});
        end

        % absolute evaluation
        [p_g_ts_all, q_g_ts_all, p_e_ts_all, q_e_ts_all, p_eg_ts_all, q_eg_ts_all] = ...
            evaluate_dataset_absolute( ground_truth, ...
            estimate_files, ...
            delay_gt_estimates(k),...
            versions, rpg );
        
        for k=1:length(colours)
            % compute RMSE of ATE
            err = p_eg_ts_all{k}-p_g_ts_all{k};
            sq = err.*err;
            sq_ate = (sum(sq,2));
            rmses(k,j) = sqrt(mean(sq_ate));
        end
    end
    for k=1:length(colours)
        % display RMSE of ATE
        err = p_eg_ts_all{k}-p_g_ts_all{k};
        sq = err.*err;
        sq_ate = (sum(sq,2));
        rmse = sqrt(mean(sq_ate));
        %ate = sqrt(sum(sq,2));
        %mate = mean(ate)
        %versions_rmse{k+1} = strcat(versions{k});
        fprintf('|-%s %f\n', versions{k}, median(rmses(k,:)));
        results(i,k) = median(rmses(k,:));
    end
end

disp(results)
disp(mean(results,1))
