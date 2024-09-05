function [q_gt_resampled] = ... 
    resample_quaternion(q_e, timestamp_e, q_gt, timestamp_gt)

q_gt_resampled = zeros(size(q_e));



for i=1:size(timestamp_e,1)
    t_est = timestamp_e(i);
    %the condition returns 1 if the t_gt is less the t_e
    %summing those we find the index of the last stamp that preceeds the
    %estimated one.
    idx = sum(timestamp_gt<=t_est);
    t_min = timestamp_gt(idx);
    % the stamp that succeeds the estimation stamp is the next element
    % in timestamp_gt
    if(idx+1 > length(timestamp_gt))
        q_gt_resampled(i,:) = q_gt(idx,:);
        break;
    end
    t_max = timestamp_gt(idx+1);
    q_start = q_gt(idx,:);
    q_end = q_gt(idx+1,:);
    fraction = (t_est - t_min)/(t_max - t_min); % wrong
    q_gt_resampled(i,:) = quatinterp(q_start,q_end,double(fraction),'slerp');
end

