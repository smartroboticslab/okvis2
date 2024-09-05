function [ p_e,q_e,p_g,q_g, T_ge] = ... 
    align_trajectories( p_e, q_e, p_g, q_g)
%ALIGN_TRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here

% compute centroid for estimate
e_centroid = zeros(1,3);

for i=1:size(p_e,1)
    e_centroid = e_centroid + p_e(i,:);
end
e_centroid = e_centroid / size(p_e,1);

%compute centroid for ground truth
g_centroid = zeros(1,3);

for i=1:size(p_g,1)
    g_centroid = g_centroid + p_g(i,:);
end
g_centroid = g_centroid / size(p_g,1);

% compute new estimate trajectory
estimate_normalized = zeros(size(p_e));
for i=1:size(p_e,1)
    estimate_normalized(i,:) = p_e(i,:) - e_centroid;
end

% compute new ground truth trajectory
groundtruth_normalized = zeros(size(p_g));
for i=1:size(p_g,1)
    groundtruth_normalized(i,:) = p_g(i,:) - g_centroid;
end

% compute matrix H
H = zeros(3);

% no roll/pitch alignment
%estimate_normalized(:,3) = 0*estimate_normalized(:,3);
%groundtruth_normalized(:,3) = 0*groundtruth_normalized(:,3);
for i = 1:size(p_g,1)
    H = H + estimate_normalized(i,:)'*groundtruth_normalized(i,:);
end

% Compute SVD
[U,~,V] = svd(H);

% build solution
X = V*U';
if det(X) == -1
    V(1:3,3) = -V(1:3,3);
    X = V*U';
end

R_ge = X;
t_ge = g_centroid' - R_ge*e_centroid';

% apply transformation to estimate
for i=1:size(p_e,1)
    p_e(i,:) = (R_ge*p_e(i,:)' + t_ge )';
end

% rotate also the quaternions
q_ge = rotm2quat(R_ge);
q_e = quatmultiply(q_ge, q_e);


% compose the transformation
T_ge = eye(4);
T_ge(1:3,1:3) = R_ge;
T_ge(1:3,4) = t_ge;


end
