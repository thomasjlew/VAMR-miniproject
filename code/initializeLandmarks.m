function [P,X] = initializeLandmarks(kf1,kf2,K)
% Initializes a first set of landmarks from two manually selected keyframes
% step1: extract keypoint-descriptors from first and second keyframe
% step2: match keypoints across frames
% step3: apply RANSAC filter to reject outliers and estimate Fundamental
% step4: triangulate landmarks
%
% kf1: [HxW] intensity img first keyframe
% kf2: [HxW] intensity img second keyframe
% K: [3x3] camera matrix
%
% P: [2xN] set of 2D keypoints from second keyframe
% X: [3xN] set of 3D landmarks where the i-th landmark corresponds
% to the i-th keypoint in kf2

%% step1: extract keypoint-descriptors from first and second keyframe

%% step2: match keypoints across frames

%% step3: apply RANSAC filter to reject outliers and estimate Fundamental

%% step4: triangulate landmarks

P=zeros(2,1);
X=zeros(3,1);

end

