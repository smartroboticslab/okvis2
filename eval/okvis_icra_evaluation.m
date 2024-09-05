%% VIO Evaluation
% written by lestefan 07/2013

clear variables global
close all


%% configurable paramenters - edit

%dataset names
dataset_names = ...
    {'vicon_room_01_01', 'vicon_room_01_02', 'vicon_room_01_03',...
    'vicon_room_02_01', 'vicon_room_02_02'};
% 
% dataset_names = ...
%     {'vicon_room_01_01'};


% folders
dataset_folder = ...
    '/media/anicastro/3130e427-7e55-46f5-8b99-b98ca0c6dbbf/datasets/euroc_dataset/';

data_folder = '/home/anicastro/Documents/okvis_data/data_vanilla/';

result_folder = '/home/anicastro/Documents/okvis_data/results/';
result_folder = [result_folder char(datetime('now')) ];
%filenames
gt_file = 'mav0/state_groundtruth_estimate0/data.csv';
est_file_folder = '/states';

% estimator versions
versions = {'vio'};

%groundtruth delay w.r.t. the estimator output
delay_gt_estimates = [0.2, 0.2, 0.2, -0.2, -0.2];

%% evaluate dataset
statistics = cell(size(dataset_names,2), 4);

for i=1:size(dataset_names,2)
    name = dataset_names{i};
    fprintf('Evaluating dataset %s with estimator versions ', name);
    for k=1:size(versions,2)
        fprintf('%s ', versions{k});
    end
    fprintf('\n');
    
    ground_truth = [dataset_folder name '/' gt_file];
    fprintf('\tGround truth file: %s\n', ground_truth);
    
    estimate_files = cell(size(versions));
    for k=1:size(versions,2)
        files  = dir([data_folder versions{k} '/' name est_file_folder '/' ]);
        files = {files.name};
        files = natsortfiles(files);
        estimate_files{k} = [data_folder versions{k} '/' name est_file_folder '/' files{end}];
        fprintf('\tFile with estimated states: %s\n', estimate_files{k});
    end
    
    %generate plots and statistics
%     [plots, stats] = evaluate_dataset_relative(...
      evaluate_dataset_relative(...
        ground_truth, ...
        estimate_files, ...
        delay_gt_estimates(k),...
        name, ...
        versions);
%     close all;
    % add the stats to the cell array
%     statistics{i,1} = dataset_names{i};
%     statistics{i,2} = versions;
%     statistics{i,3} = {'ATE', 'Orientation Error', 'Z-Axis Orientation Error'};
%     statistics{i,4} = stats;
    
%     
%         figure;
%     plot_statistics(statistics, gcf);
end

% destination file name
% mkdir(result_folder);
% result_file = [result_folder '/' 'results.tex'];

% write_statistics_table(result_file, statistics);
