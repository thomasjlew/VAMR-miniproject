%% Setup
clear;
clc;
close all;

% ds = 0; % 0: KITTI, 1: Malaga, 2: parking
% ds = 1; % 0: KITTI, 1: Malaga, 2: parking
ds = 2; % 0: KITTI, 1: Malaga, 2: parking

%% Establish kitti dataset
if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    kitti_path = '../data/kitti';
    assert(exist(kitti_path, 'dir') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    
    % specify frame count for initialization keyframes
    bootstrap_frames=[1, 3];
    
    % -------- Parameters KITTI ---------
    % initialization parameters KITTI
    params_initialization = struct (...
    ...% Harris detection parameters
    'MinQuality', 25e-5, ...             
    'FilterSize', 11, ...
    'MaxRatio', 0.8, ...
    ... % Feature extraction parameters
    'BlockSizeHarris', 11, ...                
    'Unique', true,...
    ... % Feature matching parameters
    'NumTrials', 3000, ...              
    'DistanceThreshold', 0.2, ...
    ... % KLT tracking parameters
    'BlockSizeKLT',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',inf ...  
    );
    % processFrame parameters KITTI
    params_continouos = struct (...
    ... % KLT parameters
    'BlockSize',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1,... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
    ... % P3P and RANSAC parameters 
    'MaxNumTrials',3000, ...            
    'Confidence',65,...
    'MaxReprojectionError', 6, ...
    'AlphaThreshold', 8 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
    ... % Harris paramters for canditate keypoint exraction
    'MinQuality', 15e-5, ... % higher => less keypoints
    'FilterSize', 9, ...
    ... % Matching parameters for duplicate keypoint removal
    'BlockSizeHarris', 21, ... % feature extraction parameters
    'MaxRatio', 0.99,... % higehr => more matches
    'MatchThreshold', 100.0,...  % lower  => less  
    'Unique', false, ...
    ... % Minimum pixel distance between new candidates and existing keypoints
    'MinDistance', 10 ...
    );

%% Establish malaga dataset
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = '../data/malaga';
    assert(exist(malaga_path, 'dir') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    
    % specify frame count for initialization keyframes
    bootstrap_frames=[1, 3];
    
    % -------- Parameters MALAGA ---------
    % initialization parameters Malaga
    params_initialization = struct (...
    ...% Harris detection parameters
    'MinQuality', 25e-5, ...             
    'FilterSize', 11, ...
    'MaxRatio', 0.8, ...
    ... % Feature extraction parameters
    'BlockSizeHarris', 11, ...                
    'Unique', true,...
    ... % Feature matching parameters
    'NumTrials', 3000, ...              
    'DistanceThreshold', 0.2, ...
    ... % KLT tracking parameters
    'BlockSizeKLT',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',inf ...  
    );
    % processFrame parameters Malaga
    params_continouos = struct (...
    ... % KLT parameters
    'BlockSize',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1,... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
    ... % P3P and RANSAC parameters 
    'MaxNumTrials',3000, ...            
    'Confidence',65,...
    'MaxReprojectionError', 6, ...
    'AlphaThreshold', 8 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
    ... % Harris paramters for canditate keypoint exraction
    'MinQuality', 15e-5, ... % higher => less keypoints
    'FilterSize', 9, ...
    ... % Matching parameters for duplicate keypoint removal
    'BlockSizeHarris', 21, ... % feature extraction parameters
    'MaxRatio', 0.99,... % higehr => more matches
    'MatchThreshold', 100.0,...  % lower  => less  
    'Unique', false, ...
    ... % Minimum pixel distance between new candidates and existing keypoints
    'MinDistance', 40 ...
    );

%% Establish parking dataset
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = '../data/parking';
    assert(exist(parking_path,'dir') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    
    % specify frame count for initialization keyframes
    bootstrap_frames=[1, 5];
    
    % -------- Parameters PARKING ---------
    % initialization parameters Parking 
    params_initialization = struct (...
    ...% Harris detection parameters
    'MinQuality', 2e-5, ...             
    'FilterSize', 3, ...
    'MaxRatio', 0.8, ...
    ... % Feature extraction parameters
    'BlockSizeHarris', 11, ...                
    'Unique', true,...
    ... % Feature matching parameters
    'NumTrials', 3000, ...              
    'DistanceThreshold', 0.2, ...
    ... % KLT tracking parameters
    'BlockSizeKLT',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',inf ...  
    );
    % processFrame parameters Parking 
    params_continouos = struct (...
    ... % KLT parameters
    'BlockSize',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1,... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
    ... % P3P and RANSAC parameters 
    'MaxNumTrials',3000, ...            
    'Confidence',65,...
    'MaxReprojectionError', 6, ...
    'AlphaThreshold', 8 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
    ... % Harris paramters for canditate keypoint exraction
    'MinQuality', 15e-5, ... % higher => less keypoints
    'FilterSize', 9, ...
    ... % Matching parameters for duplicate keypoint removal
    'BlockSizeHarris', 21, ... % feature extraction parameters
    'MaxRatio', 0.99,... % higehr => more matches
    'MatchThreshold', 100.0,...  % lower  => less  
    'Unique', false, ...
    ... % Minimum pixel distance between new candidates and existing keypoints
    'MinDistance', 10 ...
    );
else
    assert(false);
end

%% Bootstrap (Initialization)
% load two manually selected keyframes from dataset
if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

% initialize first set of landmarks using two-view SfM
[P_initial,X_initial,R_initial,t_initial] = ...
        initializeLandmarksHarris(img0,img1,K,params_initialization);

% initialize camera poses
cam_poses(:,:,1) = [eye(3,3),[0;0;0]];
cam_poses= cat(3,[R_initial,t_initial']);
locations = t_initial;

% initalize Markox state variables to start continouos operation
% prev_state = struct('P',P_initial,'X',X_initial,'C',zeros(2,1),'F',zeros(2,1),'T',zeros(12,1));
prev_state = struct('P',P_initial,'X',X_initial,'C',zeros(1,2),'F',zeros(2,1),'T',zeros(12,1));
prev_img = img1;


%% Intialize plots
% plot initial set of 3d landpoints and origin
f_cameraPose = figure('Name','Feature Extraction');
    set(gcf, 'Position', [800, 300, 500, 500],'InnerPosition',[0, 0, 500, 500]) 

% plot inital camera pose and landmarks
f_cameraTrajectory = figure('Name','3D camera trajectory');
    % set window position and size [left bottom width height]
    set(gcf, 'Position', [0, 300, 800, 500])
    xlim([-10,50]); ylim([-10,20]); zlim([-10,50]);
    % set viewpoint
    view(0, 0);
    set(gca, 'CameraUpVector', [0, 0, 1]);
    xlabel('x-axis, in meters');ylabel('y-axis, in meters');zlabel('z-axis, in meters'); 
    grid on
    hold on
    % plot
    cameraSize = 2;
    comOrigin = plotCamera('Size', cameraSize, 'Location',...
        [0 0 0], 'Orientation', eye(3),'Color', 'g', 'Opacity', 0);
    camInitialization = plotCamera('Size', cameraSize, 'Location',...
        t_initial, 'Orientation', R_initial,'Color', 'r', 'Opacity', 0);
    trajectory = plot3(0, 0, 0, 'b-');
    legend('Estimated Trajectory');
    title('Camera trajectory');
    % plot 3D landmarks
    legend('Initial 3D landmarks');
    scatter3(X_initial(:, 1), X_initial(:, 2), X_initial(:, 3), 5); hold on; grid on;
    legend('AutoUpdate','off');

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;

for i = range
    fprintf('\n\nProcessing frame %d\n=====================', i);
    %% Load next image from dataset
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    
    %% Process next frame and store new camera pose
    [state,pose] = processFrame(prev_state,prev_img,image,params_continouos,K,f_cameraPose);
    cam_poses = cat(3,cam_poses,pose);
    locations = cat(1,locations,pose(:,4)');
    
    %% Plot current camera pose and newly triangulated landmarks
%     figure(f_cameraPose);
%     subplot(2,2,1);
%     % plotCamera('Location', t', 'Orientation', R, 'Opacity', 0, 'Color', [0,1,0]);
%     % construct arrow to visualie camera pose
%     p1=t; p2=t+R*[0;0;3];
%     mArrow3(p1,p2, 'stemWidth', 0.05);
%     % scatter3(prev_t(1),prev_t(2),prev_t(3),'b+');
%     % legend('3d pts', '1st cam', '2nd cam');
% 
%     % Plot newly triangulated keypoints as well (SOME BUGS FOR NOW !)
%     % scatter3(state.X(:, 1), state.X(:, 2), state.X(:, 3), 5); hold on; grid on;
    
    %% Plot camera trajectory
    figure(f_cameraTrajectory);
        % plot the estimated trajectory.
        set(trajectory, 'XData', locations(:,1), 'YData', ...
        locations(:,2), 'ZData', locations(:,3));
        % plot landmarks
        newKeypoints = state.X(~ismember(state.X,prev_state.X,'rows'),:);
        if ~isempty(newKeypoints)
            scatter3(newKeypoints(1), newKeypoints(2), newKeypoints(3), 5); hold on; grid on;
        end

    %% Update input varibles for next iteration
    prev_img = image;
    prev_state = state;
    
end