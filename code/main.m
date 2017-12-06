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
    
    % parameters
    params_initialization = struct (...
    'MinQuality', 1e-5, ...             % Harris detection parameters
    'FilterSize', 3, ...
    'MaxRatio', 0.8, ...
    'BlockSizeHarris', 11, ...                % Feature extraction parameters
    'Unique', true,...
    'NumTrials', 3000, ...              % Feature matching parameters
    'DistanceThreshold', 0.2, ...
    'BlockSizeKLT',[21 21], ...            % KLT parameters
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',inf ...  
    );

    params_continouos = struct (...
    'BlockSize',[21 21], ...            % KLT parameters
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',inf, ...    
    'MaxNumTrials',3000, ...            % P3P and RANSAC parameters 
    'Confidence',50, ...
    'MaxReprojectionError', 6 ...
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
    
    % malaga parameters
    params_initialization = struct (...
    'MinQuality', 1e-5, ...             % Harris detection parameters
    'FilterSize', 3, ...
    'MaxRatio', 0.8, ...
    'BlockSizeHarris', 11, ...                % Feature extraction parameters
    'Unique', true,...
    'NumTrials', 3000, ...              % Feature matching parameters
    'DistanceThreshold', 0.2, ...
    'BlockSizeKLT',[21 21], ...            % KLT parameters
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',inf ...  
    );

    params_continouos = struct (...
    'BlockSize',[21 21], ...            % KLT parameters
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',inf, ...    
    'MaxNumTrials',3000, ...            % P3P and RANSAC parameters 
    'Confidence',50, ...
    'MaxReprojectionError', 6 ...
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
    
    % parking parameters
    params_initialization = struct (...
    'MinQuality', 1e-5, ...             % Harris detection parameters
    'FilterSize', 3, ...
    'MaxRatio', 0.8, ...
    'BlockSizeHarris', 11, ...                % Feature extraction parameters
    'Unique', true,...
    'NumTrials', 3000, ...              % Feature matching parameters
    'DistanceThreshold', 0.2, ...
    'BlockSizeKLT',[21 21], ...            % KLT parameters
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',inf ...  
    );

    params_continouos = struct (...
    ... % KLT parameters
    'BlockSize',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1,...%inf, ... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
    ... % P3P and RANSAC parameters 
    'MaxNumTrials',3000, ...            
    'Confidence',70,...%50, ...
    'MaxReprojectionError', 6, ...
    'AlphaThreshold', 5 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
    ... % Harris detection parameters
    'MinQuality', 1e-5, ...    
    'FilterSize', 3, ...
    'MaxRatio', 0.99,...%0.8, ...   % bigger => more matches
    'MatchThreshold', 100.0,...      % lower  => less
    'BlockSizeHarris', 11, ...                % Feature extraction parameters
    'Unique', false ...%true,...
    );
else
    assert(false);
end

%% Bootstrap
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

% initalize Markox state variables to start continouos operation
% prev_state = struct('P',P_initial,'X',X_initial,'C',zeros(2,1),'F',zeros(2,1),'T',zeros(12,1));
prev_state = struct('P',P_initial,'X',X_initial,'C',zeros(1,2),'F',zeros(2,1),'T',zeros(12,1));
prev_img = img1;

% initialize figure to show matches and to plot pose
figure_KLT = figure('Name','Keypoint matches - KLT');

% plot initial set of 3d landpoints and origin
f_cameraPose = figure('Name','3D camera trajectory');
xlabel('x-axis, in meters');ylabel('y-axis, in meters');zlabel('z-axis, in meters'); 
xlim([-10,50]); ylim([-10,20]); zlim([-10,50]);
hold on; grid on; 
% figure(f_cameraPose);
scatter3(X_initial(:, 1), X_initial(:, 2), X_initial(:, 3), 5); hold on; grid on;
plotCamera('Location', [0,0,0], 'Orientation', eye(3,3), 'Opacity', 0, 'Color', [1,0,0]); 
scatter3(0,0,0,'r+');
plotCamera('Location', t_initial, 'Orientation', R_initial, 'Opacity', 0, 'Color', [0,1,0]);
%scatter(t_initial(1),t_initial(2),t_initial(3),'g+');
%legend('3d initialization landmarks', '1st cam pose', '2nd cam pose');

% Save camera poses
cam_poses = zeros(3,4,2);
cam_poses(:,:,1) = [eye(3,3),[0;0;0]];
cam_poses(:,:,2) = [R_initial,t_initial'];

%% Continuous operation
n_frames = 100;
range = (bootstrap_frames(2)+1):bootstrap_frames(2)+n_frames;
% range = (bootstrap_frames(2)+1):last_frame;
cam_poses = zeros(3,4,n_frames);

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
    [state,pose] = processFrame(prev_state,prev_img,image,params_continouos,K,figure_KLT);
    cam_poses(:,:,i) = pose;
    R = cam_poses(1:3,1:3,i); t = cam_poses(1:3,4,i);
    
    %% Plot current camera pose and newly triangulated landmarks
    figure(f_cameraPose);
    % plotCamera('Location', t', 'Orientation', R, 'Opacity', 0, 'Color', [0,1,0]);
    % construct arrow to visualie camera pose
    p1=t; p2=t+R*[0;0;3];
    mArrow3(p1,p2, 'stemWidth', 0.05);
    % scatter3(prev_t(1),prev_t(2),prev_t(3),'b+');
    % legend('3d pts', '1st cam', '2nd cam');

    % Plot newly triangulated keypoints as well (SOME BUGS FOR NOW !)
    scatter3(state.X(:, 1), state.X(:, 2), state.X(:, 3), 5); hold on; grid on;
    
    %% Update input varibles for next iteration
    prev_img = image;
    prev_state = state;
    
end