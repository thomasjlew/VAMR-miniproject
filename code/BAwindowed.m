function [poseAdjusted,XAdjusted] = BAwindowed(windowLength,camOrientations,camLocations,cameraParams,X,F_P,pose_in)
%BUNDEADJUSMENT Summary of this function goes here
%   Detailed explanation goes here

%% hiddenState
cameraRotations = zeros(3,3,windowLength);
cameraTranslations = zeros(windowLength,3);
hiddenState = [];

% twists
% convert pose history to camera extrinsics
for i = 1:windowLength
    %TODO: change order in which camera poses are saved so we dont have to
    %invert the index. Currently stored the last index (most recent) to
    %first (first pose)
    [cameraRotations(:,:,i),cameraTranslations(i,:)] = cameraPoseToExtrinsics(camOrientations(:,:,end-windowLength+i),camLocations(end-windowLength+i,:));
    twist = HomogMatrix2twist([cameraRotations(:,:,i),cameraTranslations(i,:)';ones(1,4)]);
    hiddenState = [hiddenState,twist'];
end   

% landmarks
X_temp = X';
hiddenState = [hiddenState,X_temp(:)'];
%% obeservations
observations = [windowLength, length(X)];
for i = 1:windowLength
    p = []; l = [];
    numObservedX = length(X);
    for j = 1:length(F_P)
        if size(F_P{j},1) >= i
            p = [p, flip(F_P{j}(end-i+1,:),2)];
            l = [l, j];
%             % Debug plot
%             repX = worldToImage(cameraParams,cameraRotations(:,:,end),cameraTranslations(end,:),X(j,:));
%             point = F_P{j}(end-i+1,:);
%             scatter(repX(:,1),repX(:,2))
%             scatter(point(:,1),point(:,2))
%             drawnow();
        else
            numObservedX = numObservedX-1;
        end
    end
    observation_i = [numObservedX, p , l];
    observations = [observations, observation_i];
end

options = optimoptions(@lsqnonlin, 'Display', 'iter', 'MaxIter', 10);
if false % pattern
    options.JacobPattern = pattern;
    options.UseParallel = false;
end
BAerror(hiddenState,observations,cameraParams,windowLength)
errorX = @(hiddenState) BAerror(hiddenState, observations, cameraParams, windowLength);
hiddenStateAdjusted = lsqnonlin(errorX, cast(hiddenState,'double'), [], [], options);

% Update optimized poses
twistAdjusted = hiddenStateAdjusted((windowLength-1)*6+1 : windowLength*6)';
poseAdjusted = twist2HomogMatrix(twistAdjusted);
% Update optimized landmarks:
XAdjusted = reshape(hiddenStateAdjusted(6*windowLength+1:end), 3, [])';

end

