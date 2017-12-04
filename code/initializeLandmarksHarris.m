function [P,X,R,t] = initializeLandmarksHarris(kf1,kf2,K,params)
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
% P: [2xN] set of 2D keypoints FROM SECOND KEYFRAME
% X: [3xN] set of 3D landmarks where the i-th landmark corresponds
% R: [3x3] rotation matrix corresp. to 2nd camera
% t: [1x3] translation vector' corresp. to 2nd camera
% to the i-th keypoint in kf2

fprintf('Starting initialization ... \n');

%% step1: extract keypoint-descriptors from first and second keyframe
% Keypoint detection
P1 = detectHarrisFeatures(kf1,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);
P2 = detectHarrisFeatures(kf2,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);
fprintf('\n detected %d keypoints in keyframe 1', length(P1));
fprintf('\n detected %d keypoints in keyframe 2 \n', length(P2));
% Harris feature extraction
%[descriptors1,P1_valid] = extractFeatures(kf1,P1,'BlockSize',params.BlockSizeHarris);
%[descriptors2,P2_valid] = extractFeatures(kf2,P2,'BlockSize',params.BlockSizeHarris);

%% step2: match keypoints across frames
% KLT tracking
tracker = vision.PointTracker('BlockSize',params.BlockSizeKLT,'MaxIterations',params.MaxIterations, ...
    'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
initialize(tracker,P1.Location,kf1);
[P2, validIdx] = step(tracker, kf2);
P1_matched = P1.Location(validIdx, :);
P2_matched = P2(validIdx, :);
% Harris matching
%indexPairs = matchFeatures(descriptors1,descriptors2,'Unique',params.Unique,'MaxRatio',params.MaxRatio);
% select only the matched keypoints
%P1_matched = P1_valid(indexPairs(:,1),:);
%P2_matched = P2_valid(indexPairs(:,2),:);
fprintf('\n matched %d keypoints across keyframes \n', length(P1_matched));

%% step3: apply RANSAC filter to reject outliers and estimate Fundamental
[F,inliersIndex] = estimateFundamentalMatrix(P1_matched,P2_matched,...
    'Method','RANSAC','NumTrials',params.NumTrials,'DistanceThreshold',params.DistanceThreshold);
% select only matched keypoints that were not rejected by RANSAC
P1_inliers = P1_matched(inliersIndex,:);
P2_inliers = P2_matched(inliersIndex,:);
fprintf('\n after RANSAC: %d remaining matches \n', length(P1_inliers));
% Plot matches
figure('Name','Keypoint matches after RANSAC'); 
showMatchedFeatures(kf1,kf2,P1_inliers,P2_inliers);

% construct cameraIntrinsics object that can be passed to function
% relativeCameraPose()
intrinsics = cameraParameters('IntrinsicMatrix',K'); %%CAREFUL: TRANSPOSE!
% calculate relative rot. and translation from camera poses in kf1 to kf2
[orient,location] = relativeCameraPose(F,intrinsics,P1_inliers,P2_inliers);
[R, t] = cameraPoseToExtrinsics(orient, location);

%% step4: triangulate landmarks
M1 = cameraMatrix(intrinsics,eye(3),[0 0 0]); %K*[R;t]  => 4x3
M2 = cameraMatrix(intrinsics,R,t);

X = triangulate(P1_inliers,P2_inliers,M1,M2);
P = P2_inliers;

% DEBUG
% f_cameraPose = figure('Name','3D camera trajectory');
% xlabel('x-axis, in meters');ylabel('y-axis, in meters');zlabel('z-axis, in meters'); 
% hold on; grid on;
% % figure(f_cameraPose);
% scatter3(X(:, 1), X(:, 2), X(:, 3), 5); hold on; grid on;
% % DEBUG

fprintf('... Accomplished initialization \n');


%% Additionnal functions to automaticly arrange figures
disp('------------------------------------------------------------------');
disp('Arrange figures to display nicely onto monitor');
% autoArrangeFigures(0,0,1);

end


