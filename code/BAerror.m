function errorX = BAerror(hiddenState, observations, cameraParams, nFrames)
%ERRORFUNCTION Summary of this function goes here
%   Detailed explanation goes here

% 6xnum_frames matrix; stores twists in columns
twists = reshape(hiddenState(1:nFrames*6), 6, []); 
% 3xnum_frames matrix; stores landmarks in columns
X = reshape(hiddenState(nFrames*6+1:end), 3, []); 

% give index of first observation O1 in observations (idx 1 and 2 are reseved
% for m and n)
idx = 3;

errorX = [];
for i = 1:nFrames
    % recover homogeneos tranformation of current frame from twist
    i
    T = twist2HomogMatrix(twists(:, i))
    k_i = observations(idx);
    R = T(1:3,1:3);
    t = T(1:3,4);
    
    % extract keypoints P from observations
    P = flip(reshape(observations(idx+1 : idx+k_i*2), 2, []))';
    
    % extract landmarks from obervations (only those observed in this frame)
    indexesXinThisFrame = observations(idx+k_i*2+1 : idx+k_i*3);   
    X_frame = X(:,indexesXinThisFrame);
    
    % reproject landmarks to image plane
    reprojections = worldToImage(cameraParams, R, t , X_frame');

%     % Debug plot
%     figure; hold on
%     scatter(reprojections(:,1),reprojections(:,2),'+','b');
%     scatter(P(:,1),P(:,2),'+','r');
%     hold off;

    % cat reporjection errors to error matrix
    errorX = [errorX P'-reprojections'];
    
    % increase index to point at next observation Oi
    idx = idx + k_i*3 + 1;
end

errorX  = cast(errorX,'double');

end

