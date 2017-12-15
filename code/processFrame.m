function [state,pose,inlierShare] = processFrame(prev_state,prev_img,current_img,params,cameraParams,f_trackingP,IsKeyframe,live_plotting)
% Processes a new frame by calculating the updated camera pose as well as
% an updated set of landmarks
% step1: track keypoints from previous image and select the corresponding landmarks
% steC: estimate the updated camera pose from 2D-3D correspondences
% step3: acquire N new keypoint candidates and add them to state.C [2x(M+N)]
% step4: for every of N candidates write the current camera pose to state.T [12x(M+N]
% step5: evaluate track for every keypoint candidate state.F_C [2x(M+N)]
% step6: triangulate landmarks for all keypoints with sufficient track length and sufficient base line
% step7: move candidate keypoints from state.C to state.P
%
% prev_state: [1x5] struct array containing state of prev_frame
%       prev_state.P: [Nx2] keypoints
%       prev_state.X: [Nx3] landmarks ordered corresponding to keypoints
%       prev_state.C: [Mx2] candidate keypoints to be evaluated
%       prev_state.F_C: [Mx2] observed tracks for candidate keypoints
%       prev_state.T: [Mx12] camera poses at first observation of
%       candidates
% prev_img: [HxW] last processed intensity img
% current_img: [HxW] intensity img
% params: [1x?] struct array containing all tuning parameters set in main.n
% K: [3x3] camera cameraParams
%
% state: [1x5] struct array containing the new state of the processed frame
% pose: [3x4] transformation matrix [R|t] corresponding to the current camera pose

% create empty struct for current state
state = struct ('P', [],'F_P',{{}},'X',[],'C',[],'F_C',{{}},'T',[]);

%% step1.1: track keypoints in state.P and select the corresponding landmarks
% construct and initialize KLT point tracker
pointTracker = vision.PointTracker('BlockSize',params.BlockSize,'MaxIterations',params.MaxIterations, ...
    'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
initialize(pointTracker,prev_state.P,prev_img);
% use KLT point tracker to track keypoints from previous frame
[P,indexes_tracked] = step(pointTracker,current_img);
state.P = P(indexes_tracked,:);
state.X = prev_state.X(indexes_tracked,:);
state.F_P = prev_state.F_P(indexes_tracked,:);

% add current coordinates to keypoint tracks
for i = 1:length(state.P)
    state.F_P{i} = [state.F_P{i};state.P(i,:)];
end

%% step1.2: track potential keypoints in state.C
% Remove lost (non tracked) keypoints.
% construct and initialize KLT point tracker
if ~isempty(prev_state.C) % Don't try to track non existent features (1st run)
    pointTracker = vision.PointTracker('BlockSize',params.BlockSize,'MaxIterations',params.MaxIterations, ...
        'NumPyramidLevels',params.NumPyramidLevels,'MaxBidirectionalError',params.MaxBidirectionalError);
    initialize(pointTracker,prev_state.C,prev_img);
    % use KLT point tracker to track keypoints from previous frame
    [C_tracked,indexes_tracked] = step(pointTracker,current_img);
    % update candidate coordinates
    state.C = C_tracked(indexes_tracked,:);
    % remove non tracked features
    state.F_C = prev_state.F_C(indexes_tracked,:);
    state.T = prev_state.T(indexes_tracked,:);
    
    % add current coordinates to keypoint tracks
    for i = 1:length(state.C)
        state.F_C{i} = [state.F_C{i};state.C(i,:)];
    end
end

%% step2: estimate the updated camera pose from 2D-3D correspondences
warningstate = warning('off','vision:ransac:maxTrialsReached');
[orientation,location,inlierIdx] = estimateWorldCameraPose(state.P,state.X,cameraParams,...
    'MaxNumTrials',params.MaxNumTrialsPnP,'Confidence',params.ConfidencePnP,'MaxReprojectionError',params.MaxReprojectionErrorPnP);
pose = [orientation,location'];
[R,t] = cameraPoseToExtrinsics(orientation,location);
% Restore the original warning state
warning(warningstate)

if live_plotting
    figure(f_trackingP);
    imshow(current_img); hold on;
    scatter(state.P(~inlierIdx,1), state.P(~inlierIdx, 2), 15, 'r','+' );
    scatter(state.P(inlierIdx,1), state.P(inlierIdx, 2), 15, 'g','+' );
    for i = 1:length(state.P)
        plot(state.F_P{i}(:,1),state.F_P{i}(:,2),'-b');
    end
    hold off;
end

inlierShare = nnz(inlierIdx)/length(state.P);

% Remove landmarks that lie behind the camera
X_camFrame = [R,t']*[state.X';ones(1,length(state.X))];
state.X = state.X(X_camFrame(3,:)>0  & X_camFrame(3,:)<50,:);
state.P = state.P(X_camFrame(3,:)>0 & X_camFrame(3,:)<50,:);
state.F_P = state.F_P(X_camFrame(3,:)>0 & X_camFrame(3,:)<50,:);

%% step3: Triangulate landmark of C and evaluate bearing angle alpha

if IsKeyframe && ~isempty(state.C)
    
    %   if alpha is above a certain threshold add the corresponding landmark to
    %   state.X and remove the candidate

    X_C = zeros(length(state.C),3);
    for i = 1:length(state.C)
        pose_F = reshape(state.T(i,:),[3,4]);
        [R_F,t_F] = cameraPoseToExtrinsics(pose_F(:,1:3),pose_F(:,4));

        M1 = cameraMatrix(cameraParams,R,t);
        M2 = cameraMatrix(cameraParams,R_F,t_F);

        [X_C(i,:),~] = triangulate(state.C(i,:),state.F_C{i}(1,:),M1,M2);      
    end
    
    % Remove landmarks that lie behind the camera
    X_C_camFrame = [R,t']*[X_C';ones(1,length(X_C))];
    X_C = X_C(X_C_camFrame(3,:)>0 & X_C_camFrame(3,:)<50,:);
    state.C = state.C(X_C_camFrame(3,:)>0 & X_C_camFrame(3,:)<50,:);
    state.F_C = state.F_C(X_C_camFrame(3,:)>0 & X_C_camFrame(3,:)<50,:);
    state.T = state.T(X_C_camFrame(3,:)>0 & X_C_camFrame(3,:)<50,:);
        
    % Move candidate landmarks with sufficient baseline to P,X
    indexesTriangulated = false(length(state.C),1);
    for i=1:length(X_C)
        % Compute angle alpha(c)
        baseline = norm(t - pose_F(:,4));
        dist_camera_to_landmark = norm(X_C(i) - pose_F(:,4)');
        alpha = 2 * (asin((baseline/2)/dist_camera_to_landmark));

        % Add triangulated keypoint if baseline large enough and remove from
        % candidates
        if abs(alpha) > params.AlphaThreshold
            if X_C(i,3)>0
                state.X = [state.X; X_C(i,:)];
                state.P = [state.P; state.C(i,:)];
                state.F_P = [state.F_P; state.F_C{i}];
            end
            indexesTriangulated(i)=true;
        end
    end
    state.C = state.C(~indexesTriangulated,:);
    state.F_C = state.F_C(~indexesTriangulated,:);
    state.T = state.T(~indexesTriangulated,:);
end

%% step4: acquire N new keypoint candidates and add them to state.C [(M+N)x2]
% state.C contains new keypoint coordinates across multiple frames

% Detect new keypoints with Harris
% C_new = detectHarrisFeatures(current_img,'MinQuality',params.MinQuality,'FilterSize',params.FilterSize);
C_new = detectSURFFeatures(current_img,'MetricThreshold', params.MetricThreshold, 'NumOctaves', ...
    params.NumOctaves, 'NumScaleLevels', params.NumScaleLevels);
C_new = selectStrongest(C_new,300);
n_keypoints = length(C_new);

% Remove pts which are matched against currently tracked keypts
% Extract Harris descriptors from keypoints state.P and keypoint candidates state.C
% [descriptors_prev, ~]   = extractFeatures(prev_img, cornerPoints([state.P;state.C]),'BlockSize',params.BlockSizeHarris); 
[descriptors_prev, ~]   = extractFeatures(prev_img, SURFPoints([state.P;state.C]),'BlockSize',params.BlockSizeHarris);  
[descriptors_new, ~] = extractFeatures(current_img, C_new, 'BlockSize',params.BlockSizeHarris);                     

% match newly detected candidates to keypoints and candiates from database
indexPairs = matchFeatures(descriptors_prev,descriptors_new,'Unique',params.Unique, ...
                                'MaxRatio',params.MaxRatio, 'MatchThreshold', params.MatchThreshold);
                            
% remove matched keypoints: we don't want to add already tracked keypoints
C_new = removerows(C_new,'ind',indexPairs(:,2));

% check if new keypoints are too close to an existing keypoint
distC = pdist2(C_new.Location,[state.P]);
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
state.F_C = [state.F_C; mat2cell(C_new.Location,ones(length(C_new.Location),1),2)]; % 1st observation of feature

% for every of N candidates write the current camera pose to state.T [(M+N)x12]
% form row vector from (3x4) pose matrix and write it N times to sate.T
T_new = repmat(reshape(pose,[1,12]),length(C_new),1);
state.T = [state.T; T_new];

% status display
fprintf('\n %d new candidates, %d remaining after duplicate removal, %d total keypoints, %d total candidates \n',...
    n_keypoints,length(C_new),length(state.P),length(state.C));
end

