function [ p_e_ts, q_e_ts, p_g_ts, q_g_ts ] = ... 
    resample_trajectories( p_e, q_e, timestamps, p_g, q_g, timestamps_gt, delay)
%RESAMPLE_TRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here


p_e_ts=timeseries(p_e,double(timestamps-timestamps(1))*1e-9);
q_e_ts=timeseries(q_e,double(timestamps-timestamps(1))*1e-9);

% compute valid data:
valid_idx = [(diff(timestamps_gt)<6e7);1];
valid_idx_ts=timeseries(valid_idx,double(timestamps_gt-timestamps(1))*1e-9-delay);
valid_idx_ts=resample(valid_idx_ts,p_e_ts.Time);

% resample ground truth
p_g_ts=timeseries(p_g,double(timestamps_gt-timestamps(1))*1e-9-delay); % apply time offset
q_g_ts=timeseries(q_g,double(timestamps_gt-timestamps(1))*1e-9-delay); % apply time offset
p_g_ts=resample(p_g_ts,p_e_ts.Time(valid_idx_ts.Data==1));
q_g_ts=resample(q_g_ts,p_e_ts.Time(valid_idx_ts.Data==1));
q_g_ts.Data = quatnormalize(q_g_ts.Data);


end

