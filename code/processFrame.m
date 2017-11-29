function [state,pose] = processFrame(prev_state,prev_img,current_img,params,K)
% Processes a new frame by calculating the updated camera pose as well as
% an updated set of landmarks
% step1: track keypoints from previous image and select the corresponding landmarks
% step2: estimate the updated camera pose from 2D-3D correspondences
% step3: acquire N new keypoint candidates and add them to state.C [2x(M+N)]
% step4: for every of N candidates write the current camera pose to state.T [12x(M+N]
% step5: evaluate track for every keypoint candidate state.F [2x(M+N)]
% step6: triangulate landmarks for all keypoints with sufficient track length and sufficient base line
% step7: move candidate keypoints from state.C to state.P
%
% prev_state: [1x5] struct array containing state of prev_frame
%       prev_state.P: [2xN] keypoints
%       prev_state.X: [3xN] landmarks ordered corresponding to keypoints
%       prev_state.C: [2xM] candidate keypoints to be evaluated
%       prev_state.F: [2xM] observed tracks for candidate keypoints
%       prev_state.T: [12xM] camera poses at first observation of
%       candidates
% prev_img: [HxW] last processed intensity img
% current_img: [HxW] intensity img
% params: [1x?] struct array containing all tuning parameters set in main.n
% K: [3x3] camera intrinsics
%
% state: [1x5] struct array containing the new state of the processed frame
% pose: [3x4] transformation matrix [R|t] corresponding to the current camera pose

% create empty struct for current state
state = struct ('P', zeros(2,1),'X',zeros(3,1),'C',zeros(2,1),'F',zeros(2,1),'T',zeros(12,1));

%% step1: track keypoints from previous image and select the corresponding landmarks
% construct and initialize KLT point tracker
pointTracker = vision.PointTracker('BlockSize',params.BlockSize,'MaxIterations',params.MaxIterations, ...
    'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
initialize(pointTracker,prev_state.P,prev_img);
% use KLT point tracker to track keypoints from previous frame
[P,indexes_tracked] = step(pointTracker,current_img);
state.P = P(indexes_tracked,:);
state.X = prev_state.X(indexes_tracked,:);

%% step2: estimate the updated camera pose from 2D-3D correspondences
intrinsics = cameraParameters('IntrinsicMatrix',K);
[R,t] = estimateWorldCameraPose(state.P,state.X,intrinsics,...
    'MaxNumTrials',params.MaxNumTrials,'Confidence',params.Confidence,'MaxReprojectionError',params.MaxReprojectionError);
pose = [R,t'];

%% step3: acquire N new keypoint candidates and add them to state.C [2x(M+N)]
%TODO

%% step4: for every of N candidates write the current camera pose to state.T [12x(M+N]
%TODO

%% step5: evaluate track for every keypoint candidate state.F [2x(M+N)]
%TODO

%% step6: triangulate landmarks for all keypoints with sufficient track length and sufficient base line
%TODO

%% step7: move candidate keypoints from state.C to state.P
%TODO

end

