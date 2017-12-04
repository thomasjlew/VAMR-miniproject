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
state = struct ('P', zeros(2,1),'X',zeros(3,1),'C',[],'F',[],'T',[]);
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

%% Step1.2 Track potential keypoints in state.C
%   This will remove lost (non tracked) keypoints.
% construct and initialize KLT point tracker
if prev_state.C~=[0,0] % Don't try to track non existent features (1st run)
    pointTracker = vision.PointTracker('BlockSize',params.BlockSize,'MaxIterations',params.MaxIterations, ...
        'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
    initialize(pointTracker,prev_state.C,prev_img);
    % use KLT point tracker to track keypoints from previous frame
    [C_tracked,indexes_tracked] = step(pointTracker,current_img);
    % Remove non tracked features
    state.C = C_tracked(indexes_tracked,:);
    state.F = prev_state.F(indexes_tracked,:);
    state.T = prev_state.T(:,indexes_tracked);
end

% figure(figure_KLT); 
% showMatchedFeatures(prev_img,current_img,prev_state.P(indexes_tracked,:),state.P);
% hold on;

%% step2: estimate the updated camera pose from 2D-3D correspondences
intrinsics = cameraParameters('IntrinsicMatrix',K');
[R,t] = estimateWorldCameraPose(state.P,state.X,intrinsics,...
    'MaxNumTrials',params.MaxNumTrials,'Confidence',params.Confidence,'MaxReprojectionError',params.MaxReprojectionError);
pose = [R,t'];

%% step3: acquire N new keypoint candidates and add them to state.C [2x(M+N)]'
% state.C contains new keypoint tracks across multiple frames
% ! doesn't include points tracked in state.P (2d pts) (eq. to state.X(3d))
% ! include new tracks (=> save them in state.F later)

% Detect new keypoints with Harris
P2 = detectHarrisFeatures(current_img,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);

% -------------------------------------------------------------
% Remove pts which are matched against currently tracked keypts  --- - -- -- - @OPTIMIZATION: Add other checks, such as 2d distance with other pts big enough
% -------------------------------------------------------------
% Extract Harris descriptors                                  tracked3dpts;candidates 
[descriptors_prev, ~]   = extractFeatures(prev_img, cornerPoints([state.P;state.C]));  %% @OPTIMIZATION: save descriptors
[descriptors_new, valid_corners] = extractFeatures(current_img, P2);                                  % and dont compute them each time
% Match
indexPairs = matchFeatures(descriptors_prev,descriptors_new,'Unique',params.Unique, ...
                                'MaxRatio',params.MaxRatio, 'MatchThreshold', params.MatchThreshold);
% REMOVE the matched points: we don't want to add same tracked keypoints
not_matched_idx = not(ismember(P2.Location,P2(indexPairs(:,2)).Location)); %[Matched_nbx2]
P2_NOT_matched = P2(not_matched_idx(:,1));

% plot results (new extracted keypoints)
scatter(P2_NOT_matched.Location(:,1),P2_NOT_matched.Location(:,2),'b+');

% Add new keypoints to potentially future triangulated features
% state.C = [prev_state.C;P2_NOT_matched.Location];
% state.F = [prev_state.F;P2_NOT_matched.Location]; % 1st observation of feature
state.C = [state.C;P2_NOT_matched.Location];
state.F = [state.F;P2_NOT_matched.Location]; % 1st observation of feature
fprintf('\n New added keypoints: %d \n', length(P2_NOT_matched));
disp(' ');
% -------------------------------------------------------------


%% step4: for every of N candidates write the current camera pose to state.T [12xM]
% state.T = [prev_state.T, repmat(pose(:),1,size(P2_NOT_matched.Location,1))];
state.T = [state.T, repmat(pose(:),1,size(P2_NOT_matched.Location,1))];

%% step5: evaluate track for every keypoint candidate state.F [2x(M+N)]
intrinsics = cameraParameters('IntrinsicMatrix',K');
% We don't do RANSAC here since we already know [R,t]. Fix that maybe
% [orient,location] = relativeCameraPose(F,intrinsics,P1_inliers,P2_inliers); % <<<<<----@OPTIMIZATION: IMPROVE PRECISION HERE --------
% [R, t] = cameraPoseToExtrinsics(orient, location);

% Indices of tracks to be kept since baseline too small
idx_keep = []; % 1 => pt to stay in {state.C & state.F & state.T}
               % 0 => to be removed (moved to state.X & state.P)

% @OPTIMIZATION - remove for loop ------------------------------------
% create index vector corresp. to matches with same state.C & state.F
%               and triangulate these pts together
% @OPTIMIZATION ------------------------------------------------------
for i=1:size(state.C,1)%-newly added pts)   %% @OPTIMIZATION: Don't process newly added pts: their baseline is 0 
    % Extract position & orientation
    Rt1 = reshape(state.T(:,i),3,4); % [R,t] , same as "pose"
    % Save camera matrix
    M1 = cameraMatrix(intrinsics,Rt1(1:3,1:3),Rt1(1:3,4)'); %K*[R;t]  => 4x3
    M2 = cameraMatrix(intrinsics,pose(1:3,1:3),pose(1:3,4)');
    
    % Triangulate each keypoint candidate
    pt_3d = triangulate(state.F(i,:),state.C(i,:),M1,M2);  % PROBLEM HERE: TRIANGULATED PTS ARE BEHIND CAMERA?!
                                                            % IDEA: estimate [R,t] using some pts with relativeCameraPose 
                                                            %       and RANSAC fundamental matrix computation
                                                            %       and compare with the one normally computed with odometry
    % Compute angle alpha(c)
    baseline =      pose(1:3,4)' - Rt1(1:3,4)';
    dist_to_pt_3d = pt_3d        - Rt1(1:3,4)';
    % By trigonometry, we have: sin(alpha/2) = (baseline/2)/dist_to_pt_3d.
    alpha = 2 * (asin((baseline/2)/dist_to_pt_3d));
    
    % Add triangulated keypoint if baseline large enough
    if abs(alpha) > params.AlphaThreshold
        state.X = [state.X; pt_3d];
        state.P = [state.P; state.C(i,:)];
        
        idx_keep = [idx_keep, false];
    else
        idx_keep = [idx_keep, true];
    end    
end

%%%%%%%%% ADD SOME DEBUGGING, verify that depth of pts are in front of camera for example
%%%%%%%%% and other ideas. (Now, Problem: some points are behind the camera)

% Remove successfully triangulated keypoints
state.C = state.C(logical(idx_keep),:);
state.F = state.F(logical(idx_keep),:);
state.T = state.T(:,logical(idx_keep));

disp(' ');
fprintf('Added %d new 3d pts', sum(idx_keep==0));
disp(' ');

%% step6: triangulate landmarks for all keypoints with sufficient track length and sufficient base line
%DONE
% add to state.X?

%% step7: move candidate keypoints from state.C to state.P
%DONE

end

