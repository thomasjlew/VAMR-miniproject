function [state,pose] = processFrame(prev_state,prev_img,current_img,params,K,resultDisplay)
% Processes a new frame by calculating the updated camera pose as well as
% an updated set of landmarks
% step1: track keypoints from previous image and select the corresponding landmarks
% steC: estimate the updated camera pose from 2D-3D correspondences
% step3: acquire N new keypoint candidates and add them to state.C [2x(M+N)]
% step4: for every of N candidates write the current camera pose to state.T [12x(M+N]
% step5: evaluate track for every keypoint candidate state.F [2x(M+N)]
% step6: triangulate landmarks for all keypoints with sufficient track length and sufficient base line
% step7: move candidate keypoints from state.C to state.P
%
% prev_state: [1x5] struct array containing state of prev_frame
%       prev_state.P: [Nx2] keypoints
%       prev_state.X: [Nx3] landmarks ordered corresponding to keypoints
%       prev_state.C: [Mx2] candidate keypoints to be evaluated
%       prev_state.F: [Mx2] observed tracks for candidate keypoints
%       prev_state.T: [Mx12] camera poses at first observation of
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

%% step1.1: track keypoints in state.P and select the corresponding landmarks
% construct and initialize KLT point tracker
pointTracker = vision.PointTracker('BlockSize',params.BlockSize,'MaxIterations',params.MaxIterations, ...
    'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
initialize(pointTracker,prev_state.P,prev_img);
% use KLT point tracker to track keypoints from previous frame
[P,indexes_tracked] = step(pointTracker,current_img);
state.P = P(indexes_tracked,:);
state.X = prev_state.X(indexes_tracked,:);

figure(resultDisplay);
subplot(2,1,2);
showMatchedFeatures(prev_img,current_img,prev_state.P(indexes_tracked,:),state.P);
hold on;

%% step1.2: track potential keypoints in state.C
% Remove lost (non tracked) keypoints.
% construct and initialize KLT point tracker
if ~isequal(prev_state.C,[0 0]) % Don't try to track non existent features (1st run)
    pointTracker = vision.PointTracker('BlockSize',params.BlockSize,'MaxIterations',params.MaxIterations, ...
        'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
    initialize(pointTracker,prev_state.C,prev_img);
    % use KLT point tracker to track keypoints from previous frame
    [C_tracked,indexes_tracked] = step(pointTracker,current_img);
    % update candidate coordinates
    state.C = C_tracked(indexes_tracked,:);
    % Remove non tracked features
    state.F = prev_state.F(indexes_tracked,:);
    state.T = prev_state.T(indexes_tracked,:);
end

%% step2: estimate the updated camera pose from 2D-3D correspondences
intrinsics = cameraParameters('IntrinsicMatrix',K');
[R,t] = estimateWorldCameraPose(state.P,state.X,intrinsics,...
    'MaxNumTrials',params.MaxNumTrials,'Confidence',params.Confidence,'MaxReprojectionError',params.MaxReprojectionError);
pose = [R,t'];

%% step3: evaluate track (alpha) for every keypoint candidate in state.C [(M+N)x2]
%   if alpha is above a certain threshold add the corresponding landmark to
%   state.X and remove the candidate
intrinsics = cameraParameters('IntrinsicMatrix',K');
n_landmarks = length(state.X);
indexesTriangulated = true(length(state.C),1);

% @OPTIMIZATION - remove for loop ------------------------------------
% create index vector corresp. to matches with same state.C & state.F
%               and triangulate these pts together
% @OPTIMIZATION ------------------------------------------------------
for i=1:length(state.C)
    % Extract pose at the instant when candidate C was added
    
    pose_C = reshape(state.T(i,:),[3,4]);
    
    % construct camera matrices
    M1 = cameraMatrix(intrinsics,pose_C(1:3,1:3),pose_C(1:3,4)');
    M2 = cameraMatrix(intrinsics,pose(1:3,1:3),pose(1:3,4)');
    
    % Triangulate landmark for each keypoint candidate
    landmark_C = triangulate(state.F(i,:),state.C(i,:),M1,M2);  % PROBLEM HERE: TRIANGULATED PTS ARE BEHIND CAMERA?!
                                                            % IDEA: estimate [R,t] using some pts with relativeCameraPose 
                                                            %       and RANSAC fundamental matrix computation
                                                            %       and compare with the one normally computed with odometry
    % Compute angle alpha(c)
    baseline = pose(:,4)' - pose_C(1:3,4)';
    dist_camera_to_landmark = landmark_C - pose_C(1:3,4)';
    % By trigonometry, we have: sin(alpha/2) = (baseline/2)/dist_to_pt_3d.
    alpha = 2 * (asin((baseline/2)/dist_camera_to_landmark));
    
    % Add triangulated keypoint if baseline large enough and remove from
    % candidates
    if abs(alpha) > params.AlphaThreshold
        state.X = [state.X; landmark_C];
        state.P = [state.P; state.C(i,:)];
        indexesTriangulated(i)=false;
    end
end

state.C = state.C(indexesTriangulated,:);
state.F = state.F(indexesTriangulated,:);
state.T = state.T(indexesTriangulated,:);

fprintf('\n Added %d new 3D landmarks \n', length(state.X)-n_landmarks);

%% step4: acquire N new keypoint candidates and add them to state.C [(M+N)x2]
% state.C contains new keypoint tracks across multiple frames
% ! doesn't include points tracked in state.P (2d pts) (eq. to state.X(3d))
% ! include new tracks (=> save them in state.F later)

% Detect new keypoints with Harris
C_new = detectHarrisFeatures(current_img,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);
n_keypoints = length(C_new);

% Remove pts which are matched against currently tracked keypts
% TODO @OPTIMIZATION: Add other checks, such as 2d distance with other pts big enough
% Extract Harris descriptors from keypoints state.P and keypoint candidates state.C
[descriptors_prev, ~]   = extractFeatures(prev_img, cornerPoints([state.P;state.C]),'BlockSize',params.BlockSizeHarris);  %% @OPTIMIZATION: save descriptors
[descriptors_new, ~] = extractFeatures(current_img, C_new, 'BlockSize',params.BlockSizeHarris);                                  % and dont compute them each time

% match newly detected candidates to keypoints and candiates from database
indexPairs = matchFeatures(descriptors_prev,descriptors_new,'Unique',params.Unique, ...
                                'MaxRatio',params.MaxRatio, 'MatchThreshold', params.MatchThreshold);
% remove matched keypoints: we don't want to add already tracked keypoints
C_new = removerows(C_new,'ind',indexPairs(:,2));

% check if new keypoints are too close to an existing keypoint
distC = pdist2(C_new.Location,state.P);
indicesC = true(length(C_new),1);
for i =1:length(C_new)
    % if any keypoint is close than 1 px discard candidate
    if min(distC(i,:))<params.MinDistance
        indicesC(i)=false;
    end
end
C_new = C_new(indicesC,:);

% add new keypoints to potentially future triangulated features
state.C = [state.C; C_new.Location];
state.F = [state.F; C_new.Location]; % 1st observation of feature

% for every of N candidates write the current camera pose to state.T [(M+N)x12]
% form row vector from (3x4) pose matrix and write it N times to sate.T
T_new = repmat(reshape(pose,[1,12]),length(C_new),1);
state.T = [state.T; T_new];
fprintf('%d new candidates, %d remaining after duplicate removal, %d total keypoints, %d total candidates, %d T \n',n_keypoints,length(C_new),length(state.P),length(state.C),length(state.T));
end

