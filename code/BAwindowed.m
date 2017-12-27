function [poseAdjusted,camOrientationsAdjusted,camLocationsAdjusted,XAdjusted,dError] = ...
    BAwindowed(params,camOrientations,camLocations,cameraParams,state,image)
%BUNDEADJUSMENT Summary of this function goes here
%   Detailed explanation goes here

windowLength = (params.nKeyframes-1)*params.intervalKeyframes;

%% ========================================================================
% Construct hiddenState
%==========================================================================
hiddenState = [];

% -------------------------------------------------------------------------
% Extract Twists corresponding to keyframes
% -----------------------------------------
% convert pose history to camera extrinsics
                % Take 1 cam. orientation each "intervalKeyframes"
camOrientationsCropped = camOrientations(:,:, (end-windowLength) : ...
                                                params.intervalKeyframes : ... 
                                                end);    
camLocationsCropped = camLocations( (end-windowLength)        : ...
                                     params.intervalKeyframes : ...
                                     end                      ,     :);
% -------------------------------------------------------------------------
                                 
% -------------------------------------------------------------------------
% For each keyframe, save twist into hidden state  
% -----------------------------------------------                               
for i = 1:params.nKeyframes 
    % Convert into Twist
    twist = HomogMatrix2twist(...
                [camOrientationsCropped(:,:,i), camLocationsCropped(i,:)'; ...
                [0        ,    0      ,     0 ,                        1]]);
            
    % Save into hiddenState
    hiddenState = [hiddenState,twist'];
end 
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% Save All Landmarks into Hiddenstate
% -----------------------------------
X_transpose = state.X';
hiddenState = [hiddenState, X_transpose(:)'];
% -------------------------------------------------------------------------

%% ==========================================================================
% Construct observations
%===========================================================================
observations = [];
% For each Keyframe
for frame_i = 1:params.intervalKeyframes:(windowLength+1)   % WHY "+1" ???
    P = []; l = [];                                         % WHAT IS "l" ???
    
    % Nb of observed 3d landmark (which are all saved in "state.X")
    % to check if enough landmarks to run BA (otherwise, don't run it)
    numObservedX = length(state.X);
    
    % For each tracked 3d pt (F_P is to 2d points accross subsequent frames
    %                         wich correspond to this 3d pt)
    for j = 1:length(state.F_P)
        % ?????  If track is longer that the BA windowLength ????????
        if size(state.F_P{j},1) >= frame_i 
            P = [P,flip(state.F_P{j}(frame_i,:),2)];    %% WHAT IS THIS? CORRECT INDEXING? NOT "(end-frame_i)"
            l = [l, j];                                 %  OR SMTHING LIKE THAT?
        else
            numObservedX = numObservedX-1;         
        end
    end
    % throw error of number of observed landmarks is to low for effective BA
    % assert(numObservedX/length(state.X)>=0.2, 'Error in BA: share of tracked Landmarks is below threshold');
    observation_i = [numObservedX, P , l];
    observations = [observation_i, observations];
end
observations = [params.nKeyframes, length(state.X), observations];

%% ==========================================================================
% Construct Jacobian Pattern to specify which values optimize
%===========================================================================

num_observations = (numel(observations)-2-params.nKeyframes)/3;
% Factor 2, one error for each x and y direction.
num_error_terms = 2 * num_observations;
% Each error term will depend on one pose (6 entries) and one landmark
% position (3 entries), so 9 nonzero entries per error term:
pattern = spalloc(num_error_terms, numel(hiddenState), num_error_terms * 9);

% Fill pattern for each frame individually:
observation_i = 3;  % iterator into serialized observations
error_i = 1;  % iterating frames, need another iterator for the error
for frame_i = 1:params.nKeyframes
    num_keypoints_in_frame = observations(observation_i);
    % All errors of a frame are affected by its pose.
    pattern(error_i:error_i+2*num_keypoints_in_frame-1, ...
        (frame_i-1)*6+1:frame_i*6) = 1;

    % Each error is then also affected by the corresponding landmark.
    landmark_indices = observations(...
        observation_i+2*num_keypoints_in_frame+1:...
        observation_i+3*num_keypoints_in_frame);
    for kp_i = 1:numel(landmark_indices)
        pattern(error_i+(kp_i-1)*2:error_i+kp_i*2-1,...
            1+params.nKeyframes*6+(landmark_indices(kp_i)-1)*3:...
            params.nKeyframes*6+landmark_indices(kp_i)*3) = 1;
    end

    observation_i = observation_i + 1 + 3*num_keypoints_in_frame;
    error_i = error_i + 2 * num_keypoints_in_frame;
end
% figure;
% spy(pattern);

%options = optimoptions(@lsqnonlin, 'Display', 'iter', 'MaxIter', params.maxIterations);
options = optimoptions(@lsqnonlin, 'Display', 'off', 'MaxIter', params.maxIterations, 'UseParallel',true);
options.JacobPattern = pattern;
options.UseParallel = false;

%% ==========================================================================
% Nonlinear Optimization
% ==========================================================================

% DEBUG Plot: 
% errorXbefore = BAerrorWithPlotting(hiddenState, observations, cameraParams, params.nKeyframes,image);
errorXbefore = BAerror(hiddenState, observations, cameraParams, params.nKeyframes,image);

errorX = @(hiddenState) BAerror(hiddenState, observations, cameraParams, params.nKeyframes);
hiddenStateAdjusted = lsqnonlin(errorX, cast(hiddenState,'double'), [], [], options);

% DEBUG Plot: 
% errorXafter = BAerrorWithPlotting(hiddenStateAdjusted, observations, cameraParams, params.nKeyframes,image);
errorXafter = BAerror(hiddenStateAdjusted, observations, cameraParams, params.nKeyframes,image);

dError(1) = norm(mean(errorXbefore));
dError(2) = norm(mean(errorXafter));

%% ==========================================================================
% Update state
% ==========================================================================

% Extract optimized homogeneous rotations and translation 
twistsAdjusted = reshape(hiddenStateAdjusted(1:params.nKeyframes*6)',6,[]);

% Update optimized poses
% SAVED IN WRONG ORDER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% SAVED IN WRONG ORDER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% "i" doesn't go through all length of camOrientations, only updates 1st X ones, which may not
% even have been taken into account!!!
% SAVED IN WRONG ORDER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% SAVED IN WRONG ORDER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
camOrientationsAdjusted = camOrientations;
camLocationsAdjusted = camLocations;
for i = 1:params.nKeyframes
    TAdjusted = twist2HomogMatrix(twistsAdjusted(:,i));
%     camOrientationsAdjusted(:,:,i) = TAdjusted(1:3,1:3);         %%%% OLD OLD OLD (fixed now I think?)
%     camLocationsAdjusted(i,:) = TAdjusted(1:3,4);                %%%% OLD OLD OLD (fixed now I think?)
    camOrientationsAdjusted(:,:,(end-windowLength)+(i-1)*params.intervalKeyframes) = TAdjusted(1:3,1:3);
    camLocationsAdjusted((end-windowLength)+(i-1)*params.intervalKeyframes,:)      = TAdjusted(1:3,4);
%     camOrientationsAdjusted(:,:,i) = TAdjusted(1:3,1:3)';
%     camLocationsAdjusted(i,:) = -TAdjusted(1:3,1:3)'*TAdjusted(1:3,4);
end
poseAdjusted = [camOrientationsAdjusted(:,:,end),camLocationsAdjusted(end,:)'];

% Update optimized landmarks:
XAdjusted = cast(reshape(hiddenStateAdjusted(6*params.nKeyframes+1:end), 3, [])','single');

end

