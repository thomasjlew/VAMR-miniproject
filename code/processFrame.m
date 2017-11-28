function [current_state,pose] = processFrame(prev_state,prev_frame,current_frame)
% Processes a new frame by calculating the updated camera pose as well as
% an updated set of landmarks
% step1: ...
% step2: ...
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
% prev_frame: [HxW] last processed intensity img
% current_frame: [HxW] intensity img
%
% current_state: [1x5] struct array containing state of current_frame
% pose: [3x4] transformation matrix [R|t] of the current camera pose

%% step1: ...

%% step2: ...

%% step3: ...

current_state = struct('P',zeros(2,1),'X',...
    zeros(3,1),'C',zeros(2,1),'F',zeros(2,1),'T',zeros(12,1));
pose = [eye(3), zeros(3,1)];

end

