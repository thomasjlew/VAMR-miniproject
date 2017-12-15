function [poseAdjusted,camOrientationsAdjusted,camLocationsAdjusted,XAdjusted] = BAwindowed(windowLength,camOrientations,camLocations,cameraParams,X,F_P,iterations,img)
%BUNDEADJUSMENT Summary of this function goes here
%   Detailed explanation goes here

%% Construct hiddenState
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
%% Construct obeservations
observations = [windowLength, length(X)];
for i = 1:windowLength
    p = []; l = [];
    numObservedX = length(X);
    % Debug plot
%     hold off
%     figure; hold on
    for j = 1:length(F_P)
        if size(F_P{j},1) >= i
            p = [p, flip(F_P{j}(end-i+1,:),2)];
            l = [l, j];
            % Debug plot
%             repX = worldToImage(cameraParams,cameraRotations(:,:,end),cameraTranslations(end,:),X(j,:));
%             point = F_P{j}(end-i+1,:);         
%             scatter(repX(:,1),repX(:,2),'+','g')
%             scatter(point(:,1),point(:,2),'+','r')
%             drawnow();
        else
            numObservedX = numObservedX-1;
        end
    end
    observation_i = [numObservedX, p , l];
    observations = [observations, observation_i];
end

%options = optimoptions(@lsqnonlin, 'Display', 'iter', 'MaxIter', iterations);
options = optimoptions(@lsqnonlin, 'Display', 'iter');
if false % pattern
    options.JacobPattern = pattern;
    options.UseParallel = false;
end

%% Nonlinear Optimization
errorX = @(hiddenState) BAerror(hiddenState, observations, cameraParams, windowLength);
hiddenStateAdjusted = lsqnonlin(errorX, cast(hiddenState,'double'), [], [], options);

%% Update state
% Extract optimized homogeneous rotations and translation 
twistsAdjusted = reshape(hiddenStateAdjusted(1:windowLength*6)',6,[]);
TAdjusted = zeros(4,4,windowLength);

% Update optimized poses
camOrientationsAdjusted = camOrientations;
camLocationsAdjusted = camLocations;
for i = 1:windowLength
    TAdjusted(:,:,i) = twist2HomogMatrix(twistsAdjusted(:,i));
    [OrientationAdjusted,LocationsAdjusted] = ...
        extrinsicsToCameraPose(TAdjusted(1:3,1:3,i),TAdjusted(1:3,4,i));
    camOrientationsAdjusted(:,:,end-windowLength+i) = OrientationAdjusted;
    camLocationsAdjusted(end-windowLength+i,:) = LocationsAdjusted;
end
poseAdjusted = [camOrientationsAdjusted(:,:,end),camLocationsAdjusted(end,:)'];
% Update optimized landmarks:
XAdjusted = cast(reshape(hiddenStateAdjusted(6*windowLength+1:end), 3, [])','single');

end

