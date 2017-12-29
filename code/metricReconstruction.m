function [scaledState,scaledCamLocations,scalingFactor,stateMR] = metricReconstruction(state,camLocations,pose,prev_stateMR,prev_img,image,params,cameraParams)
%METRIC_RECONSTRUCTION
%   This function will scan the dataset for markers (checkerBoards). Given
%   the ground truth metric dimension of the marker it will scale the scene
%   accordingly to align the scale to ground truth. 
%   
%   state: [1x6] struct array containing state of prev_frame
%       state.P: [Nx2] keypoints
%       state.F_P: [Nx1] cell array containing tracks of keypoints
%       state.X: [Nx3] landmarks ordered corresponding to keypoints
%       state.C: [Mx2] candidate keypoints to be evaluated
%       state.F_C: [Mx2] tracks for candidate keypoints
%       state.T: [Mx12] camera poses at first observation of keypoints
%   camLocations: [frameCountx3] array containing the camera locations in
%   each frame
%   pose: [3x4] current camera pose
%   prev_stateMR: [1x5] struct array containing the state of Metric
%   Reconstruction
%       stateMR.P: [9x2] tracked checkerBoard keypoints
%       stateMR.X: [9x3} landmarks corresponding to currently tracked
%       checkerBoard
%       stateMR.F: [9x2] first observation of tracked keypoints
%       stateMR.T:  [9x12] poses of camera at first observation of P
%       stateMR.alpha: [9x1] array containing the most recent angles of X
%
% This funtion has three different modes of operation
% SEARCH CHECKERBOARD: If no checkerBoard is currently tracked the function
%   will try to detect any visible checkerboard and add cornerPoints to the
%   current keypoints
% TRACK CHECKERBOARD: As soon as there is a checkerBoard detected the
%   function will stop scanning for new checkerboards and will track the
%   current checkerBoard until the bearing angles alpha reach a certain
%   threshold
% RECONSTRUCT SCENE: If now all bearing angles alpha are above the given
%   threshold the function will use the triangulated landmarks to re-scale
%   the scene. Then all points and landmarks are removed and the function
%   switches back to searchCheckerBoard

stateMR = prev_stateMR;

%% Define reference distance of checkerboard corners
cornerDist = 0.2; % ground truth distance between two neighboring corners in [m]
nCorners = 9;
nEdges = sqrt(nCorners)-1;
[x,y] = meshgrid(0:cornerDist:nEdges*cornerDist,0:cornerDist:nEdges*cornerDist);
% calculate the mean of all pairwise distances between the checkerboard corners
% this is a robust measure for the scale of the checkerboard and will
% remain the same even if the corners are saved in a different order
refDistance = mean(pdist(x+y));

%% Intialize mode of operation (see detailed explanation in function head)
searchCheckerBoard = false;
trackCheckerBoard = false;
reconstructScene = false;

%% Set output variables for the case that no rectification is performed
scaledState = state;
scaledCamLocations = camLocations;
scalingFactor = 1;

%% Operation Mode: SCENE RECONSTRUCTION
% check if the past checkerboardvlandmarks can be triangulated with
% sufficient accuracy by checking their bearing angles alpha
if mean(stateMR.alpha)>params.AlphaThreshold
    reconstructScene = true;
end

if reconstructScene
    figure(6);
    imshow(image); hold on
    scatter(stateMR.P(:,1), stateMR.P(:, 2), 15, 'g','+' ); hold off
    figure(7);
    scatter3(stateMR.X(:,1),stateMR.X(:,2), stateMR.X(:, 3), 15, 'g','o' );
    
    % calculate the mean of all pairwise distances between the 9
    % checkerboard landmarks
    measuredDistance = mean(pdist(state.X));
    
    % re-scale scene only if the number of cornes is correct. Otherwise
    % discard landmarks and keypoints and return state without scaling
    % this is to prevent any falsely detected patterns
    if size(stateMR.X,1)==nCorners
        % re-scale landmarks and cam trajectory to fit ground truth
        scalingFactor = refDistance/measuredDistance;
        scaledState = state;
        scaledState.X = state.X * scalingFactor;
        scaledCamLocations = camLocations * scalingFactor;
    else
        scaledState = state;
    end
    
    % discard the tracked checkerBoard points and landmarks to initiate the
    % next cycle of searching, tracking and scaling
    stateMR.P = [];
    stateMR.F = [];
    stateMR.T = [];
    stateMR.X = [];
    stateMR.alpha = 0;
end

%% Determine if there is a checkerBoard currently tracked
% if no scan image for new checkerBoard
if isempty(stateMR.P)
    searchCheckerBoard = true;
% if yes track the checker corners and triangulate landmarks
else
    trackCheckerBoard = true;
end

%% Operation Mode: SEARCH CHECKERBOARD
% scan images for checkerboard and add new keypoint candidates if checkerBoard is detected
if searchCheckerBoard
    warning('off','all');
    [checkerPoints,~]=detectCheckerboardPoints(image);
    warning('on','all');

    if ~isempty(checkerPoints)
        stateMR.P = checkerPoints;
        stateMR.F = checkerPoints;
        stateMR.alpha = zeros(size(stateMR.P,1),1);
        % for all new candidates write the current camera pose to stateMR.T [(M+N)x12]
        % form row vector from (3x4) pose matrix and write it N times to sate.T
        stateMR.T = repmat(reshape(pose,[1,12]),length(stateMR.P),1);
    end
end

%% Operation Mode: TRACK CHECKERBOARD
if trackCheckerBoard
    pointTracker = vision.PointTracker('BlockSize',params.BlockSize,'MaxIterations',params.MaxIterations, ...
    'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
    
    % use KLT point tracker to track cornerPoints from previous frame
    initialize(pointTracker,stateMR.P,prev_img);
    [trackedCornerPoints,indexes_tracked] = step(pointTracker,image);
    stateMR.P = trackedCornerPoints(indexes_tracked,:);
    release(pointTracker);
    
    figure(6);
    imshow(image); hold on;
    scatter(stateMR.P(:,1), stateMR.P(:, 2), 15, 'r','+' );
    
    % triangulate landmarks using the first and the most recent
    % observation, then calculate alpha for each of the cornerPoints
    for i = 1:size(stateMR.P,1)
        pose_F = reshape(stateMR.T(i,:),[3,4]); % extract pose at first observation from stateMR.T
        [R,t] = cameraPoseToExtrinsics(pose(:,1:3),pose(:,4));
        [R_F,t_F] = cameraPoseToExtrinsics(pose_F(:,1:3),pose_F(:,4));
        M1 = cameraMatrix(cameraParams,R,t);
        M2 = cameraMatrix(cameraParams,R_F,t_F);

        % triangulate landmark using pose from current and first observation
        [stateMR.X(i,:),~] = triangulate(stateMR.P(i,:),stateMR.F(i,:),M1,M2);      

        % Compute bearing vectors at first and most current observations
        X_currentCamFrame = R'*stateMR.X(i,:)'   + t';
        X_firstCamFrame =   R_F'*stateMR.X(i,:)' + t_F';
        % compute angle in radians between bearing vectors
        stateMR.alpha(i) = atan2( norm(cross(X_firstCamFrame,X_currentCamFrame)) , ...
            dot(X_firstCamFrame,X_currentCamFrame) ); 
    end
end

end

