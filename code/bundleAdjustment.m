function pose = bundleAdjustment(camOrientations,camLocations,cameraParams,X,F_P,pose_in)
%BUNDEADJUSMENT Summary of this function goes here
%   Detailed explanation goes here

nFramesOptimized = 5;
cameraRotations = zeros(3,3,nFramesOptimized);
cameraTranslations = zeros(nFramesOptimized,3);

% convert pose history to camera extrinsics
for i = 1:nFramesOptimized
    %TODO: change order in which camera poses are saved so we dont have to
    %invert the index. Currently stored the last index (most recent) to
    %first (first pose)
    [cameraRotations(:,:,i),cameraTranslations(i,:)] = cameraPoseToExtrinsics(camOrientations(:,:,end-i+1),camLocations(end-i+1,:));
end   

% select only keypoints with tracks across atleast 5 images and discard all
% earlier trackings
indexesThresholded = false(length(F_P),1);
for i = 1:length(F_P)
    if length(F_P{i})>=(nFramesOptimized)
        indexesThresholded(i) = true;
        F_P{i} = F_P{i}((end-nFramesOptimized+1):end,:);
    end
end
X = X(indexesThresholded,:);
F_P = F_P(indexesThresholded,:);

% extract coordinates of keypoints in the current fram (Pminus0) and the 4
% prior frames (Pminus...)
PointTracks = cell2mat(F_P);
P = zeros(length(F_P),2,nFramesOptimized);
for i = 1:nFramesOptimized
    P(:,:,i) = PointTracks((nFramesOptimized+1-i):nFramesOptimized:length(PointTracks),:);
end

% for every of the last nFramesOptimized frames calculate reprojection
% errors
Errors = zeros(length(F_P),nFramesOptimized);
for i = 1:nFramesOptimized
    Differences = P(:,:,i) - worldToImage(cameraParams,cameraRotations(:,:,i),...
        camLocations(i,:),X);
    Errors(:,i) = sqrt(Differences(:,1).^2+Differences(:,1).^2);
end

pose = pose_in;
end

