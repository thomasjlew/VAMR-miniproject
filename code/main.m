%% Setup
clear;
clc;
close all;

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
    bootstrap_frames=[1, 4];
    
    % -------- Parameters KITTI ---------
    % initialization parameters KITTI
    params_initialization = struct (...
    ...% Harris detection parameters
    'MinQuality', 15e-5, ...               
    'FilterSize', 15, ...
    ... % Feature matching parameters
    'NumTrials', 3000, ...              
    'DistanceThreshold', 0.2, ...
    ... % KLT tracking parameters
    'BlockSizeKLT',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1 ...  
    );
    % processFrame parameters KITTI
    params_continouos = struct (...
    ... % KLT parameters
    'BlockSize',[21 21], ...            
    'MaxIterations',20, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1,... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
    ... % P3P parameters 
    'MaxNumTrialsPnP',1500, ...            
    'ConfidencePnP',95,...
    'MaxReprojectionErrorPnP', 3, ...
    ... % Triangulation parameters
    'AlphaThreshold', 3 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
    ... % Harris paramters for canditate keypoint exraction
    'MinQuality', 15e-5, ... % higher => less keypoints
    'FilterSize', 15, ...
    ... % Matching parameters for duplicate keypoint removal
    'BlockSizeHarris', 21, ... % feature extraction parameters
    'MaxRatio', 1.00,... % higehr => more matches
    'MatchThreshold', 100.0,...  % higher  => more matches  
    'Unique', false, ...
    ... % Minimum pixel distance between new candidates and existing keypoints
    'MinDistance', 6 ...
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
    bootstrap_frames=[1, 4];
    
    % -------- Parameters MALAGA ---------
    % initialization parameters Malaga
    params_initialization = struct (...
    ...% Harris detection parameters
    'MinQuality', 25e-5, ...             
    'FilterSize', 15, ...
    ... % Feature matching parameters
    'NumTrials', 3000, ...              
    'DistanceThreshold', 0.2, ...
    ... % KLT tracking parameters
    'BlockSizeKLT',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',6 ...  
    );
    % processFrame parameters Malaga
    params_continouos = struct (...
    ... % KLT parameters
    'BlockSize',[21 21], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1,... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
    ... % P3P parameters 
    'MaxNumTrialsPnP',1000, ...            
    'ConfidencePnP',95,...
    'MaxReprojectionErrorPnP', 2, .....
    ... % Triangulation parameters
    'AlphaThreshold', 8 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
    ... % Harris paramters for canditate keypoint exraction
    'MinQuality', 20e-5, ... % higher => less keypoints
    'FilterSize', 11, ...
    ... % Matching parameters for duplicate keypoint removal
    'BlockSizeHarris', 11, ... % feature extraction parameters
    'MaxRatio', 0.99,... % higehr => more matches
    'MatchThreshold', 100.0,...  % lower  => less  
    'Unique', false, ...
    ... % Minimum pixel distance between new candidates and existing keypoints
    'MinDistance', 15 ...
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
    'MinQuality', 1e-5, ...             
    'FilterSize', 11, ...
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
    ... % P3P parameters 
    'MaxNumTrialsPnP',3000, ...            
    'ConfidencePnP',95,...
    'MaxReprojectionErrorPnP', 3, ...
    ... % Triangulation parameters
    'AlphaThreshold', 8 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
    ... % Harris paramters for canditate keypoint exraction
    'MinQuality', 5e-5, ... % higher => less keypoints
    'FilterSize', 15, ...
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
[P_initial,X_initial,orientation_inital,location_initial] = ...
        initializeLandmarksHarris(img0,img1,K,params_initialization);    
    
% initialize camera poses
camOrientations = orientation_inital;
camLocations = location_initial;

% initalize Markox state variables to start continouos operation
prev_state = struct('P',P_initial,'X',X_initial,'C',[],'F',[],'T',[],'L',[]);
prev_img = img1;


%% Intialize plots
% plot initial set keypoints and krypoint tracks
f_trackingP = figure('Name','Feature Tracking Keypoints');
    set(gcf, 'Position', [800, 1000, 500, 500])
% f_trackingC = figure('Name','Feature Tracking Keypoint Candidates');
%     set(gcf, 'Position', [800, 0, 500, 500])

% plot inital camera pose and landmarks
f_cameraTrajectory = figure('Name','3D camera trajectory');
    % set window position and size [left bottom width height]
    set(gcf, 'Position', [0, 300, 800, 500])
    xlim([-70,70]); ylim([-10,20]); zlim([-10,100]);
    % set viewpoint
    view(0, 0);
    set(gca, 'CameraUpVector', [0, 0, 1]);
    xlabel('x-axis, in meters');ylabel('y-axis, in meters');zlabel('z-axis, in meters'); 
    grid on
    hold on
    % plot camera
    cameraSize = 1.5;
    comOrigin = plotCamera('Size', cameraSize, 'Location',...
        [0 0 0], 'Orientation', eye(3),'Color', 'r', 'Opacity', 0.2);
    cam = plotCamera('Size', cameraSize, 'Location',location_initial, ...
        'Label','Current Pose', 'Orientation', orientation_inital,'Color', 'black', 'Opacity', 0.2);
    trajectory = plot3(0, 0, 0, 'black-','LineWidth',3);   
    title('Camera trajectory');
    % plot 3D landmarks
    landmarks_scatter = scatter3(X_initial(:, 1), X_initial(:, 2), X_initial(:, 3), 8, 'o','b'); grid on;
    legend('Estimated Trajectory');
    legend('AutoUpdate','off');

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
IsKeyframe = false;

for i = range
    fprintf('\n Processing frame %d\n=====================', i);
    %% Load next image from dataset
    if ds == 0 %KITTI
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
        figure(f_cameraTrajectory);
        xlim([-20,70]); ylim([-10,20]); zlim([-5,100]);
    elseif ds == 1 %Malaga
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
        figure(f_cameraTrajectory);
        xlim([-40,40]); ylim([-10,20]); zlim([-5,100]);
    elseif ds == 2 %Parking
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
        figure(f_cameraTrajectory);
        xlim([-5,70]); ylim([-10,20]); zlim([-5,50]);
    else
        assert(false);
    end
    
    %% Process next frame and store new camera pose
    % triangulate new keypoints only every 5th frame
    if mod(i,3)==0
        IsKeyframe=true;
    else
        IsKeyframe=false;
    end
    [state,pose] = processFrame(prev_state,prev_img,image,params_continouos,K,f_trackingP,IsKeyframe);
    camOrientations = cat(3,camOrientations,pose(:,1:3));
    camLocations = cat(1,camLocations,pose(:,4)');

    %% Plot camera trajectory
    figure(f_cameraTrajectory);
        % plot the estimated trajectory.
        set(trajectory, 'XData', smooth(camLocations(:,1)), 'YData', ...
        smooth(camLocations(:,2)), 'ZData', smooth(camLocations(:,3)));
        cam.Location = camLocations(end,:);
        cam.Orientation = camOrientations(:,:,end);
        % plot landmarks
        newKeypoints = state.X(~ismember(state.X,prev_state.X,'rows'),:);
        if ~isempty(newKeypoints)
            landmarks_scatter.XData = state.X(:,1);
            landmarks_scatter.YData = state.X(:,2);
            landmarks_scatter.ZData = state.X(:,3);
        end
%     figure(f_trackingC);
%         showMatchedFeatures(prev_img,image,state.F,state.C);

    %% Update input varibles for next iteration
    prev_img = image;
    prev_state = state;
    
end