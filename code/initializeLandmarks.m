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

% num_kps = ;
% img1 = imread(); uint8
% img2 = imread();

%% step1: extract keypoint-descriptors from first and second keyframe
%% Parameters for harris detector from ex 3

har_psz = ;
har_kappa = ;  %0.04 to 0.15
har_nmax_supression_r = ;
har_desc_r = ;
har_match_lambda = ;

% har_scores1 = functionname(img1, har_psz, har_kappa); %double
% har_scores2 = functionname(img2, har_psz, har_kappa); %double
% select keypoints
% har_kp1= functionname(har_scores1, num_kps, har_nmax_supressions_r); %[2XN]
% P = functionname(har_scores2, num_kps, har_nmax_supressions_r); %[2XN]

% describe keypoints
% har_desc1 = functionname(img1, har_kp1,har_desc_r);

%% step2: match keypoints across frames
% har_match = functionname(har_desc1, har_desc2, har_match_lambda);

%% step3: apply RANSAC filter to reject outliers and estimate Fundamental

%% step4: triangulate landmarks

P=zeros(2,1);
X=zeros(3,1);

end

