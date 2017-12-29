function [scaledState,scaledCamLocations,stateMR] = metricReconstruction(state,camLocations,pose,prev_stateMR,prev_img,image,params,cameraParams)
%METRIC_RECONSTRUCTION
%   This function will scan the dataset for markers (checkerBoards). Given
%   the ground truth metric dimension of the marker it will scale the scene
%   accordingly to align the scale to ground truth
%   

%% reference distance of checkerboard corners
actualDistance = 0.2;
edgeLengths = 2;
[x,y] = meshgrid(0:actualDistance:edgeLengths*actualDistance,0:actualDistance:edgeLengths*actualDistance);
refDistance = mean(pdist(x+y));

stateMR = prev_stateMR;

rectifyScene = false;
searchCheckerBoard = false;
trackCheckerBoard = false;

if mean(stateMR.alpha)>params.AlphaThreshold
    rectifyScene = true;
else
    scaledState = state;
    scaledCamLocations = camLocations;
end

if rectifyScene
    %% Scale 3D scene 
    figure(6);
    imshow(image); hold on
    scatter(stateMR.P(:,1), stateMR.P(:, 2), 15, 'g','+' ); hold off
    figure(7);
    scatter3(stateMR.X(:,1),stateMR.X(:,2), stateMR.X(:, 3), 15, 'g','o' );
    
    measuredDistance = mean(pdist(state.X));
    
    if size(stateMR.X,1)==9
        scalingFactor = refDistance/measuredDistance
        scaledState = state;
        scaledState.X = state.X * scalingFactor;
        scaledCamLocations = camLocations * scalingFactor;
    else
        scaledState = state;
    end
    
    stateMR.P = [];
    stateMR.F = [];
    stateMR.T = [];
    stateMR.X = [];
    stateMR.alpha = 0;
end

if isempty(stateMR.P)
    searchCheckerBoard = true;
else
    trackCheckerBoard = true;
end

if searchCheckerBoard
    %% Add new keypoint candidates if checkerBoard is detected
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

if trackCheckerBoard
    %% Track checkerBoard points
    pointTracker = vision.PointTracker('BlockSize',params.BlockSize,'MaxIterations',params.MaxIterations, ...
    'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
    
    initialize(pointTracker,stateMR.P,prev_img);
    % use KLT point tracker to track cornerPoints from previous frame
    [trackedCornerPoints,indexes_tracked] = step(pointTracker,image);
    stateMR.P = trackedCornerPoints(indexes_tracked,:);
    release(pointTracker);
    
    figure(6);
    imshow(image); hold on;
    scatter(stateMR.P(:,1), stateMR.P(:, 2), 15, 'r','+' );
    
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

