function errorX = BAerrorWithPlotting(hiddenState, observations, cameraParams, nFrames,image)
%ERRORFUNCTION Summary of this function goes here
%   Detailed explanation goes here

% 6xnum_frames matrix; stores twists in columns
twists = reshape(hiddenState(1:nFrames*6), 6, []); 
% 3xnum_frames matrix; stores landmarks in columns
X = reshape(hiddenState(nFrames*6+1:end), 3, [])'; 

% give index of first observation O1 in observations (idx 1 and 2 are reseved
% for m and n)
idx = 3;

errorX = [];
for i = 1:nFrames
    % recover homogeneos tranformation of current frame from twists
    T = twist2HomogMatrix(twists(:, i));
    
    numKeypoints = observations(idx);
    [R,t] = cameraPoseToExtrinsics(T(1:3,1:3),T(1:3,4));
    
    % extract keypoints P from observations
    P = flip(reshape(observations(idx+1 : idx+numKeypoints*2), 2, []))';
    
    % extract landmarks from obervations (only those observed in this frame)
    indexesXinThisFrame = observations(idx+numKeypoints*2+1 : idx+numKeypoints*3);   
    X_frame = X(indexesXinThisFrame,:);
    
    % reproject landmarks to image plane
    reprojections = worldToImage(cameraParams, R, t , X_frame);
    % cat reprojection errors to error matrix
    errorsFrame = P'-reprojections';
    errorX = [errorX errorsFrame];

    % increase index to point at next observation Oi
    idx = idx + numKeypoints*3 + 1;
end

% Debug plot
figure('Name','DEBUG Bundle Adjustment');
imshow(image); hold on
scatter(reprojections(:,1),reprojections(:,2),5*sum(abs(errorsFrame))'+0.1,'o','r');
scatter(P(:,1),P(:,2),2,'+','g');
title('Keypoints (green) and reprojected Landmarks (red), The size of the circle correlates to abs of error'); 
hold off;

errorX  = cast(errorX,'double');

end

