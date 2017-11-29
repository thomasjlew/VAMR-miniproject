function [state,pose] = processFrame(prev_state,prev_img,current_img,params,K)
% Processes a new frame by calculating the updated camera pose as well as
% an updated set of landmarks
% step1: extract points from current img and match them to points from
% previous image, then select the corresponding landmarks
% previos
% step2: select landmarks of the matched keypoints to establish 2D-3D
% correspondences
% step3: ...
% step4: ...
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
%
% current_state: [1x5] struct array containing state of current_frame
% pose: [3x4] transformation matrix [R|t] of the current camera pose

% create struct for currents state
state = struct ('P', zeros(2,1),'X',zeros(3,1),'C',zeros(2,1),'F',zeros(2,1),'T',zeros(12,1));

%% step1: extract points from current img and match them to points from
% previous image, then select the corresponding landmarks
% construct and initialize KLT point tracker
pointTracker = vision.PointTracker('BlockSize',[31 31],'MaxIterations',30,'NumPyramidLevels',3);
initialize(pointTracker,prev_state.P.Location,prev_img);
% use KLT point tracker to track keypoints from previous frame
[P,indexes_tracked] = step(pointTracker,current_img);
state.P = P(indexes_tracked,:);
state.X = prev_state.X(indexes_tracked,:);

%% step2: estimate the updated camera pose from 2D-3D correspondences
intrinsics = cameraParameters('IntrinsicMatrix',K);
[R,t,inlierIdx] = estimateWorldCameraPose(state.P,state.X,intrinsics,...
    'MaxNumTrials',2000,'Confidence',99,'MaxReprojectionError',1);
 
%% step3: ...

pose = [R,t];

end

