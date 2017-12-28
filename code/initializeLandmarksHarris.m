function [P,F_P,X,orientation,location] = initializeLandmarksHarris(kf1,kf2,cameraParams,params,live_plotting)
% Initializes a first set of landmarks from two manually selected keyframes
% step1: extract keypoint-descriptors from first and second keyframe
% step2: match keypoints across frames
% step3: apply RANSAC filter to reject outliers and estimate Fundamental
% step4: triangulate landmarks
%
% kf1: [HxW] intensity img first keyframe
% kf2: [HxW] intensity img second keyframe
% K: [3x3] camera matrix
% params: struct [1x?] set of parameters for initialization algorithm
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
% P1 = detectSURFFeatures(kf1,'MetricThreshold', params.MetricThreshold, 'NumOctaves', ...
%     params.NumOctaves, 'NumScaleLevels', params.NumScaleLevels);
%P1 = selectStrongest(P1,400);
% P2 = detectSURFFeatures(kf2,'MetricThreshold', params.MetricThreshold, 'NumOctaves', ...
%     params.NumOctaves, 'NumScaleLevels', params.NumScaleLevels);
%P2 = selectStrongest(P2,400);
P1 = detectHarrisFeatures(kf1,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);
P2 = detectHarrisFeatures(kf2,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);
fprintf('\n detected %d keypoints in keyframe 1', length(P1));
fprintf('\n detected %d keypoints in keyframe 2 \n', length(P2));

%% step2: match keypoints across frames
% KLT tracking
tracker = vision.PointTracker('BlockSize',params.BlockSizeKLT,'MaxIterations',params.MaxIterations, ...
    'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
initialize(tracker,P1.Location,kf1);
[P2, validIdx] = step(tracker, kf2);
P1_matched = P1.Location(validIdx, :);
P2_matched = P2(validIdx, :);
fprintf('\n matched %d keypoints across keyframes \n', length(P1_matched));

%% step3: apply RANSAC filter to reject outliers and estimate Fundamental
[F,inliersIndex] = estimateFundamentalMatrix(P1_matched,P2_matched,...
    'Method','RANSAC','NumTrials',params.NumTrials,'DistanceThreshold',params.DistanceThreshold);
% select only matched keypoints that were not rejected by RANSAC
P1_inliers = P1_matched(inliersIndex,:);
P2_inliers = P2_matched(inliersIndex,:);
fprintf('\n after RANSAC: %d remaining matches \n', length(P1_inliers));
% Plot matches
if live_plotting
    figure('Name','Bootstrapping: Keypoint matches after RANSAC'); 
    showMatchedFeatures(kf1,kf2,P1_inliers,P2_inliers);
    set(gcf, 'Position', [800, 300, 500, 500])
end

% calculate relative rot. and translation from camera poses in kf1 to kf2
[orientation,location] = relativeCameraPose(F,cameraParams,P1_inliers,P2_inliers);
[R, t] = cameraPoseToExtrinsics(orientation, location);

%% step4: triangulate landmarks
M1 = cameraMatrix(cameraParams,eye(3),[0 0 0]); %K*[R;t]  => 4x3
M2 = cameraMatrix(cameraParams,R,t);

X = triangulate(P1_inliers,P2_inliers,M1,M2);
P = P2_inliers;
% remove landmarks that are to far away and landmarks that are behind the
% camera
distX = sqrt(sum(X.^2,2));
outlierIndixes = distX<2*mean(distX) & X(:,3)>0;
X = X(outlierIndixes, :);
P = P(outlierIndixes, :);
F_P = mat2cell(P,ones(length(P),1),2);

fprintf('... Accomplished initialization \n');

end


