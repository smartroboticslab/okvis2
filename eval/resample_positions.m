function [ p_gt_resampled] = ...
    resample_positions( p_e, timestamps_e, p_gt, timestamps_gt)
%RESAMPLE_POSITIONS Summary of this function goes here
%   Detailed explanation goes here


p_e_ts=timeseries(p_e,double(timestamps_e-timestamps_e(1))*1e-9);

% compute valid data:
valid_idx = ones(size(timestamps_gt)); %[(diff(timestamps_gt)<6e7);1];
valid_idx_ts=timeseries(valid_idx,double(timestamps_gt-timestamps_e(1))*1e-9);
valid_idx_ts=resample(valid_idx_ts,p_e_ts.Time);

% resample ground truth
p_gt_ts=timeseries(p_gt,double(timestamps_gt-timestamps_e(1))*1e-9); % apply time offset
p_gt_ts=resample(p_gt_ts,p_e_ts.Time(valid_idx_ts.Data==1));

p_gt_resampled = p_gt_ts.Data;


end

