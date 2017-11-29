function [P,X] = initializeLandmarksHarris(kf1,kf2,K,params)
% Initializes a first set of landmarks from two manually selected keyframes
% step1: extract keypoint-descriptors from first and second keyframe
% step2: match keypoints across frames
% step3: apply RANSAC filter to reject outliers and estimate Fundamental
% step4: triangulate landmarks
%
% kf1: [HxW] intensity img first keyframe
% kf2: [HxW] intensity img second keyframe
% K: [3x3] camera matrix
% params: struct [1x?] set of parameters for initialization algroithm
%       params.MinQuality
%       params.FilterSize
%       
% P: [2xN] set of 2D keypoints from second keyframe
% X: [3xN] set of 3D landmarks where the i-th landmark corresponds
% to the i-th keypoint in kf2

fprintf('Starting initialization ... \n');

%% step1: extract keypoint-descriptors from first and second keyframe
P1 = detectHarrisFeatures(kf1,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);
P2 = detectHarrisFeatures(kf2,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);
fprintf('\n detected %d keypoints in keyframe 1', length(P1));
fprintf('\n detected %d keypoints in keyframe 2 \n', length(P2));
[descriptors1,P1_valid] = extractFeatures(kf1,P1,'BlockSize',params.BlockSize);
[descriptors2,P2_valid] = extractFeatures(kf2,P2,'BlockSize',params.BlockSize);
figure('Name','Keypoints Frame 1'); imshow(kf1); hold on; plot(P1); hold off;
figure('Name','Keypoints Frame 2'); imshow(kf2); hold on; plot(P2); hold off;

%% step2: match keypoints across frames
indexPairs = matchFeatures(descriptors1,descriptors2,'Unique',params.Unique,'MaxRatio',params.MaxRatio);
% select only the matched keypoints
P1_matched = P1_valid(indexPairs(:,1),:);
P2_matched = P2_valid(indexPairs(:,2),:);
fprintf('\n matched %d keypoints across keyframes \n', length(P1_matched));
figure('Name','Keypoint matches before RANSAC'); showMatchedFeatures(kf1,kf1,P1_matched,P2_matched);

%% step3: apply RANSAC filter to reject outliers and estimate Fundamental
[F,inliersIndex] = estimateFundamentalMatrix(P1_matched,P2_matched,...
    'Method','RANSAC','NumTrials',params.NumTrials,'DistanceThreshold',params.DistanceThreshold);
% select only matched keypoints that were not rejected by RANSAC
P1_inliers = P1_matched(inliersIndex);
P2_inliers = P2_matched(inliersIndex);
fprintf('\n after RANSAC %d remaining matches \n', length(P1_inliers));
figure('Name','Keypoint matches after RANSAC'); showMatchedFeatures(kf1,kf1,P1_inliers,P2_inliers);

% construct cameraIntrinsics object that can be passed to function
% relativeCameraPose()
intrinsics = cameraParameters('IntrinsicMatrix',K);
% calculate relative rot. and translation from camera poses in kf1 to kf2
[R,t] = relativeCameraPose(F,intrinsics,P1_inliers,P2_inliers);

%% step4: triangulate landmarks
M1 = cameraMatrix(intrinsics,eye(3),zeros(3,1));
M2 = cameraMatrix(intrinsics,R,t);

X = triangulate(P1_inliers,P2_inliers,M1,M2);
P = P2_inliers;

fprintf('... Accomplished initialization \n');

end


