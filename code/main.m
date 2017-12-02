%% Setup
clear;
clc;
close all;

ds = 1; % 0: KITTI, 1: Malaga, 2: parking

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
    'BlockSize',[21 21], ...            % KLT parameters
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',inf, ...    
    'MaxNumTrials',3000, ...            % P3P and RANSAC parameters 
    'Confidence',50, ...
    'MaxReprojectionError', 6 ...
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
[P_initial,X_initial] = initializeLandmarksHarris(img0,img1,K,params_initialization);

% initalize Markox state variables to start continouos operation
prev_state = struct('P',P_initial,'X',X_initial,'C',zeros(2,1),'F',zeros(2,1),'T',zeros(12,1));
prev_img = img1;

% initialize figure to show matches and to plot pose
f_cameraPose = figure('Name','3D camera trajectory');
xlabel('x-axis');ylabel('y-axis');zlabel('z-axis');
figure_KLT = figure('Name','Keypoint matches - KLT');

% plot initial set of landpoints and origin
figure(f_cameraPose);
scatter3(X_initial(:, 1), X_initial(:, 2), X_initial(:, 3), 5);
%plotCoordinateFrame(eye(3), [0; 0; 0], 0.1);

%% Continuous operation
% only iterate over 9 images for testing purposes
range = (bootstrap_frames(2)+1):bootstrap_frames(2)+9;
% range = (bootstrap_frames(2)+1):last_frame;


for i = range
    fprintf('\n\nProcessing frame %d\n=====================', i);
    % Load next image from dataset
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
    
    % Process next frame
    [state,pose] = processFrame(prev_state,prev_img,image,params_continouos,K,figure_KLT);
    
    % Plot camera pose and landmarks
    %figure(f_cameraPose);
    %subplot(1,3,3);
    %TODO
    %R = pose(:,1:3);
    %t = pose(:,4);
    %plotCoordinateFrame(R', t, 0.7);
    %view(0,0);
    
    % Makes sure that plots refresh.    
    pause(0.01);
    
    % Update input varibles for next iteration
    prev_img = image;
    prev_state = state;
    
end
