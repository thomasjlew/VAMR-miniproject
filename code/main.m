
clear;
clc;
close all;

%% General Paramters
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
live_plotting = true;
total_frames = 93;
doBA = false;
BAwindow = 2;
maxBAiterations = 100;
KeyframeDist = 1;

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
    bootstrap_frames=[80, 84];
    
    % -------- Parameters KITTI ---------
    % initialization parameters KITTI
    params_initialization = struct (...
    ...% Harris detection parameters
    'MinQuality', 20e-5, ...               
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
    'MaxIterations',15, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1,... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
    ... % P3P parameters 
    'MaxNumTrialsPnP',1500, ...            
    'ConfidencePnP',95,...
    'MaxReprojectionErrorPnP', 3, ...
    ... % Triangulation parameters
    'AlphaThreshold', 3 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
    ... % Harris paramters for canditate keypoint exraction
    'MinQuality', 20e-5, ... % higher => less keypoints
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
    % initialization parameters MALAGA
    params_initialization = struct (...
    ...% Harris detection parameters
    'MinQuality', 20e-5, ...               
    'FilterSize', 15, ...
    ... % Feature matching parameters
    'NumTrials', 3000, ...              
    'DistanceThreshold', 0.2, ...
    ... % KLT tracking parameters
    'BlockSizeKLT',[15 15], ...            
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1 ...  
    );
    % processFrame parameters MALAGA
    params_continouos = struct (...
    ... % KLT parameters
    'BlockSize',[21 21], ...            
    'MaxIterations',15, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1,... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
    ... % P3P parameters 
    'MaxNumTrialsPnP',1000, ...            
    'ConfidencePnP',95,...
    'MaxReprojectionErrorPnP', 3, ...
    ... % Triangulation parameters
    'AlphaThreshold', 3 *pi/180, ...   %Min baseline angle [rad] for new landmark (alpha(c) in pdf)
    ... % Harris paramters for canditate keypoint exraction
    'MinQuality', 20e-5, ... % higher => less keypoints
    'FilterSize', 15, ...
    ... % Matching parameters for duplicate keypoint removal
    'BlockSizeHarris', 15, ... % feature extraction parameters
    'MaxRatio', 1.00,... % higehr => more matches
    'MatchThreshold', 100.0,...  % higher  => more matches  
    'Unique', false, ...
    ... % Minimum pixel distance between new candidates and existing keypoints
    'MinDistance', 9 ...
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

cameraParams = cameraParameters('Intrinsicmatrix',K');

% initialize first set of landmarks using two-view SfM
[P_initial,F_P_initial,X_initial,orientation_inital,location_initial] = ...
        initializeLandmarksHarris(img0,img1,cameraParams,params_initialization,live_plotting);   
    
% initialize camera poses
camOrientations = orientation_inital;
camLocations = location_initial;

% initalize Markox state variables to start continouos operation
prev_state = struct('P',P_initial,'F_P',{F_P_initial},'X',X_initial,'C',[],'F_C',{{}},'T',[]);
prev_img = img1;

% initialize other status variables
scores = 1;
keypointCount = 0;
frameCount = 0;

%% Initialize plots
if live_plotting
    f_trackingP = figure('Name','Feature Tracking Keypoints');
        set(gcf, 'Position', [800, 1000, 500, 500])
    [f_keypointScores,f_cameraTrajectory,cam,camBA,trajectory,trajectoryBA,landmarksScatter,landmarksHistoryScatter,landmarksScatterBA] = ...
        initializeFigures(location_initial,orientation_inital,X_initial);
end
%% Continuous operation
profile on

range = (bootstrap_frames(2)+1):total_frames;
IsKeyframe = false;

for i = range
    fprintf('\n Processing frame %d\n=====================', i);
    %% Load next image from dataset
    if ds == 0 %KITTI
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1 %Malaga
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2 %Parking
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    
    %% Process next frame and store new camera pose
    % triangulate new keypoints only every 5th frame
    if mod(i,KeyframeDist)==0
        IsKeyframe=true;
    else
        IsKeyframe=false;
    end
    
    [state,pose,inlierShare] = processFrame(prev_state,prev_img,image,params_continouos,...
        cameraParams,IsKeyframe,f_trackingP,live_plotting);
    
    camOrientations = cat(3,camOrientations,pose(:,1:3));
    camLocations = cat(1,camLocations,pose(:,4)');
    scores = cat(1,scores,inlierShare);
    frameCount = cat(1,frameCount,i);
    keypointCount = cat(1,keypointCount,length(state.P));
    
    %% Plot trajectory without BA
    if live_plotting
        figure(f_cameraTrajectory);
            xlim([camLocations(end,1)-20,camLocations(end,1)+20]); zlim([camLocations(end,3)-15,camLocations(end,3)+50]);
            landmarksHistoryScatter.XData = [landmarksHistoryScatter.XData state.X(:,1)'];
            landmarksHistoryScatter.YData = [landmarksHistoryScatter.YData state.X(:,2)'];
            landmarksHistoryScatter.ZData = [landmarksHistoryScatter.ZData state.X(:,3)'];
            landmarksScatter.XData = state.X(:,1);
            landmarksScatter.YData = state.X(:,2);
            landmarksScatter.ZData = state.X(:,3);
            % plot the estimated trajectory.
            set(trajectory, 'XData', smooth(camLocations(:,1)), 'YData', ...
            smooth(camLocations(:,2)), 'ZData', smooth(camLocations(:,3)));
            cam.Location = camLocations(end,:);
            cam.Orientation = camOrientations(:,:,end);
            % plot landmarks
            
        figure(f_keypointScores);  
            subplot(2,1,1);
                plot(frameCount,scores,'-');
                xlim([max([1,i-20]),i]);
            subplot(2,1,2);
                plot(frameCount,keypointCount,'-');
                xlim([max([1,i-20]),i]);
                hold off;
    end
    
    %% Sliding Window Bundle Adjustment
    if doBA   
        if i>(BAwindow+bootstrap_frames(2)+5) && IsKeyframe
        %if i == total_frames
            [poseAdjusted,camOrientationsAdjusted,camLocationsAdjusted,XAdjusted] = ...
                BAwindowed(BAwindow,camOrientations,camLocations,cameraParams,state.X,state.F_P,maxBAiterations,image);
    %         camLocations = camLocationsAdjusted;
    %         camOrientations = camOrientationsAdjusted;
%             pose = poseAdjusted;
            state.X = XAdjusted;
            % Plot REFINED camera trajectory
            if live_plotting
                figure(f_cameraTrajectory);
                    landmarksScatterBA.XData = XAdjusted(:,1);
                    landmarksScatterBA.YData = XAdjusted(:,2);
                    landmarksScatterBA.ZData = XAdjusted(:,3);
                    % plot refined estimated trajectory.
                    set(trajectoryBA, 'XData', smooth(camLocationsAdjusted(:,1)), 'YData', ...
                    smooth(camLocationsAdjusted(:,2)), 'ZData', smooth(camLocationsAdjusted(:,3)));
                    camBA.Location = camLocationsAdjusted(end,:);
                    camBA.Orientation = camOrientationsAdjusted(:,:,end);
            end
        end
    end

    %% Update input varibles for next iteration
    prev_img = image;
    prev_state = state;
    
end
profile off
%% Plot final results
if ~live_plotting
    [f_keypointScores,f_cameraTrajectory,cam,camBA,trajectory,trajectoryBA,landmarksScatter,landmarksHistoryScatter,landmarksScatterBA] = ...
        initializeFigures(location_initial,orientation_inital,[0 0 0]);
    figure(f_cameraTrajectory);
        grid on; hold on; axis equal;
        % plot the estimated trajectory.
        set(trajectory, 'XData', smooth(camLocations(:,1)), 'YData', ...
                smooth(camLocations(:,2)), 'ZData', smooth(camLocations(:,3)));
        if doBA
            set(trajectoryBA, 'XData', smooth(camLocationsAdjusted(:,1)), 'YData', ...
                    smooth(camLocationsAdjusted(:,2)), 'ZData', smooth(camLocationsAdjusted(:,3)));
        end
        
     figure(f_keypointScores);
        subplot(2,1,1);
            plot(frameCount,smooth(scores),'-');
        subplot(2,1,2);
            plot(frameCount,smooth(keypointCount),'-');
end

fprintf('\n average inlier share: %d \n', mean(scores));