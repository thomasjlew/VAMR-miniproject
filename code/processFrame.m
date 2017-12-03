function [state,pose] = processFrame(prev_state,prev_img,current_img,params,K,figure_KLT)
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
state = struct ('P', zeros(2,1),'X',zeros(3,1),'C',[],'F',zeros(2,1),'T',zeros(12,1));
% state = struct ('P', zeros(2,1),'X',zeros(3,1),'C',zeros(1,2),'F',zeros(2,1),'T',zeros(12,1));

%% step1: track keypoints from previous image and select the corresponding landmarks
% construct and initialize KLT point tracker
pointTracker = vision.PointTracker('BlockSize',params.BlockSize,'MaxIterations',params.MaxIterations, ...
    'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
initialize(pointTracker,prev_state.P,prev_img);
% use KLT point tracker to track keypoints from previous frame
[P,indexes_tracked] = step(pointTracker,current_img);
state.P = P(indexes_tracked,:);
state.X = prev_state.X(indexes_tracked,:);
figure(figure_KLT); 
showMatchedFeatures(prev_img,current_img,prev_state.P(indexes_tracked,:),state.P);
hold on;

%% step2: estimate the updated camera pose from 2D-3D correspondences
intrinsics = cameraParameters('IntrinsicMatrix',K');
[R,t] = estimateWorldCameraPose(state.P,state.X,intrinsics,...
    'MaxNumTrials',params.MaxNumTrials,'Confidence',params.Confidence,'MaxReprojectionError',params.MaxReprojectionError);
pose = [R,t'];

%% step3: acquire N new keypoint candidates and add them to state.C [2x(M+N)]'
% state.C contains new keypoints tracks across multiple frames
% ! doesn't include points tracked in state.P (2d pts) (eq. to state.X(3d))
% ! include new tracks (=> save them in state.F later)

% Detect new keypoints with Harris
P2 = detectHarrisFeatures(current_img,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);

% -------------------------------------------------------------
% Remove pts which are matched against currently tracked keypts
% -------------------------------------------------------------
% Extract Harris descriptors                                  tracked3dpts;candidates 
[descriptors_prev, ~]   = extractFeatures(prev_img, cornerPoints([state.P;state.C]));
[descriptors_new, valid_corners] = extractFeatures(current_img, P2);
% Match
indexPairs = matchFeatures(descriptors_prev,descriptors_new,'Unique',params.Unique, ...
                                'MaxRatio',params.MaxRatio, 'MatchThreshold', params.MatchThreshold);
% REMOVE the matched points: we don't want to add same tracked keypoints
not_matched_idx = not(ismember(P2.Location,P2(indexPairs(:,2)).Location)); %[Matched_nbx2]
P2_NOT_matched = P2(not_matched_idx(:,1));

% plot results (new extracted keypoints)
scatter(P2_NOT_matched.Location(:,1),P2_NOT_matched.Location(:,2),'b+');

% Add new keypoints to potentially future triangulated featuress
state.C = [prev_state.C;P2_NOT_matched.Location];
fprintf('\n New added keypoints: %d \n', length(P2_NOT_matched));
disp(' ');
% -------------------------------------------------------------


%% step4: for every of N candidates write the current camera pose to state.T [12x(M]
state.T = [prev_state.T, repmat(pose(:),1,size(P2_NOT_matched.Location,1))];

%% step5: evaluate track for every keypoint candidate state.F [2x(M+N)]
% New keypoint tracks are added to state.F (first observations of keypt)
%TODO
% for keypt
%     if newkeypt
%         state.F = [state.F, keypt];
%     end
% end
%TODO

%% step6: triangulate landmarks for all keypoints with sufficient track length and sufficient base line
%TODO
% add to state.X?

%% step7: move candidate keypoints from state.C to state.P
%TODO

end

