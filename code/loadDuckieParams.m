function [paramsInitialization,paramsContinuous] = loadDuckieParams()
%LOADKITTIPARAMS Summary of this function goes here
%   Detailed explanation goes here

% initialization parameters DUCKIE
paramsInitialization = struct (...
...% Harris detection parameters
'MinQuality', 10e-5, ...               
'FilterSize', 21, ...
... % Ransac fundamental matrix parameters
'NumTrials', 3000, ...              
'DistanceThreshold', 0.4, ...
... % KLT tracking parameters
'BlockSizeKLT',[15 15], ...            
'MaxIterations',40, ...         
'NumPyramidLevels',3, ...
'MaxBidirectionalError',1 ...  
);
% processFrame parameters DUCKIE
paramsContinuous = struct (...
... % KLT parameters
'BlockSize',[15 15], ...            
'MaxIterations',15, ...         
'NumPyramidLevels',3, ...
'MaxBidirectionalError',1,... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
... % P3P parameters 
'MaxNumTrialsPnP',1000, ...            
'ConfidencePnP',95,...
'MaxReprojectionErrorPnP', 2, ...
... % Triangulation parameters
'AlphaThreshold', 6 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
... % Harris paramters for canditate keypoint exraction
'MinQuality', 10e-5, ... % higher => less keypoints
'FilterSize', 15, ...
... % Matching parameters for duplicate keypoint removal
'BlockSizeHarris', 15, ... % feature extraction parameters
'MaxRatio', 1.00,... % higehr => more matches
'MatchThreshold', 100.0,...  % higher  => more matches  
'Unique', false, ...
... % Minimum pixel distance between new candidates and existing keypoints
'MinDistance', 6 ...
);
end

