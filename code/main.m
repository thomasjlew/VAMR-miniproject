
clear;
clc;
close all;

%% ==========================================================================
% General VO Paramters
%==========================================================================

ds = 3; % 0: KITTI, 1: Malaga, 2: parking, 3:DUCKIE
live_plotting = true;
total_frames = inf;
KeyframeDist = 1;

% BA parameters
doBA = false;
BAparams = struct(...
'nKeyframes', 4, ...
'intervalKeyframes', 3, ...
'intervalBA', 1, ...
'maxIterations', 20 ...
);

%%==========================================================================
% Establish Dataset
%==========================================================================

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
    bootstrap_frames=[78, 84];
    
    % load VO parameters
    [paramsInitialization,paramsContinuous] = loadKittiParams();

%% Establish malaga dataset
elseif ds == 1
    % Path containing the files of Malaga
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
    
    % load VO parameters
    [paramsInitialization,paramsContinuous] = loadMalagaParams();

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
    
    % load VO parameters
    [paramsInitialization,paramsContinuous] = loadParkingParams();
    

%% Establish DUCKIE dataset
elseif ds == 3
    % Path containing the many files of Duckie dataset
    duckie_path = '../vision_duckietown/duckiecalib';
    assert(exist(duckie_path, 'dir') ~= 0);
    images = dir([duckie_path ...
        '/first_total_calib_dataset']);
    last_frame = length(images);
    
    % specify frame count for initialization keyframes
    bootstrap_frames=[1, 8];
    
     % load VO parameters
    [paramsInitialization,paramsContinuous] = loadDuckieParams();
    
else
    assert(false);
end

%% ==========================================================================
% Bootstrapping (Initialization of Landmarks)
%==========================================================================

% load two manually selected keyframes from dataset
if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
    cameraParams = cameraParameters('Intrinsicmatrix',K');
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
    cameraParams = cameraParameters('Intrinsicmatrix',K');
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
    cameraParams = cameraParameters('Intrinsicmatrix',K');
elseif ds == 3
    img0 = rgb2gray(imread([duckie_path ...
        sprintf('/first_total_calib_dataset/%04d.jpg',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([duckie_path ...
        sprintf('/first_total_calib_dataset/%04d.jpg',bootstrap_frames(2))]));
    cameraCalibrator = load([duckie_path sprintf('/cameraCalibrator.mat')]);
    cameraParams = cameraCalibrator.cameraParams;
    img0 = undistortImage(img0,cameraParams);
    img1 = undistortImage(img1,cameraParams);
else
    assert(false);
end

% initialize first set of landmarks using two-view SfM
[P_initial,F_P_initial,X_initial,orientation_inital,location_initial] = ...
        initializeLandmarksHarris(img0,img1,cameraParams,paramsInitialization,live_plotting);   
    
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

%% ==========================================================================
% Initialization Plots
%===========================================================================
f_trackingP = figure('Name','Feature Tracking Keypoints');
        set(gcf, 'Position', [800, 1000, 500, 500])
if live_plotting
    [f_keypointScores,f_cameraTrajectory,cam,camBA,trajectory,trajectoryBA,landmarksScatter,landmarksHistoryScatter,landmarksScatterBA] = ...
        initializeFigures(location_initial,orientation_inital,X_initial);
end

%% ==========================================================================
% Continuous operation
%===========================================================================

range = (bootstrap_frames(2)+1):(min(total_frames,last_frame));
profile on
for i = range
    fprintf('\n Processing frame %d\n=====================', i);
    % Load next image from dataset
    if ds == 0 %KITTI
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1 %Malaga
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2 %Parking
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    elseif ds == 3 %Duckie
        image = im2uint8(rgb2gray(imread([duckie_path ...
            sprintf('/first_total_calib_dataset/%04d.jpg',i)])));
        image = undistortImage(image,cameraParams);
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
    [state,pose,inlierShare] = processFrame(prev_state,prev_img,image,paramsContinuous,...
        cameraParams,IsKeyframe,f_trackingP,live_plotting);
    
    camOrientations = cat(3,camOrientations,pose(:,1:3));
    camLocations = cat(1,camLocations,pose(:,4)');
    scores = cat(1,scores,inlierShare);
    frameCount = cat(1,frameCount,i);
    keypointCount = cat(1,keypointCount,length(state.P));
    
    %% Plot trajectory without BA
    if live_plotting
        figure(f_cameraTrajectory);
            % adjust axes to follow camera 
            xlim([camLocations(end,1)-20,camLocations(end,1)+20]); zlim([camLocations(end,3)-15,camLocations(end,3)+40]);
            % plot updated landmarks
%             landmarksHistoryScatter.XData = [landmarksHistoryScatter.XData state.X(:,1)']; 
%             landmarksHistoryScatter.YData = [landmarksHistoryScatter.YData state.X(:,2)'];
%             landmarksHistoryScatter.ZData = [landmarksHistoryScatter.ZData state.X(:,3)'];
            landmarksScatter.XData = state.X(:,1); landmarksScatter.YData = state.X(:,2); landmarksScatter.ZData = state.X(:,3);
            % plot the estimated trajectory.
            set(trajectory, 'XData', smooth(camLocations(:,1)), 'YData', ...
            smooth(camLocations(:,2)), 'ZData', smooth(camLocations(:,3)));
            cam.Location = camLocations(end,:);
            cam.Orientation = camOrientations(:,:,end);
            
        figure(f_keypointScores);  
            subplot(2,1,1);
                plot(frameCount,scores,'-');
                xlim([max([1,i-20]),i]);
                title('Share of Inlier Keypoints');
            subplot(2,1,2);
                plot(frameCount,keypointCount,'-');
                xlim([max([1,i-20]),i]);
                title('Number of tracked Keypoints');
    end
    
    %% Sliding Window Bundle Adjustment
    if doBA && mod(i-bootstrap_frames(2),BAparams.intervalBA)==0
        % do bundle adjustment only after enough frames have been processed
        if i>=(BAparams.nKeyframes*BAparams.intervalKeyframes + bootstrap_frames(2))
            fprintf('\n Init BA - keyframeInterval=%d, nKyframes=%d ...',...
                BAparams.intervalKeyframes,BAparams.nKeyframes);
            % bundle adjustment
            [poseAdjusted,camOrientationsAdjusted,camLocationsAdjusted,XAdjusted,dError] = ...
                BAwindowed(BAparams,camOrientations,camLocations,cameraParams,state,image);
            fprintf(' Finished BA: mean error before/after = %.2g / %.2g\n',dError(1),dError(2));
            
            % update landmarks
            state.X = XAdjusted;
            
            % plot adjusted landmakrs
            if live_plotting
                figure(f_cameraTrajectory);
                    landmarksScatterBA.XData = XAdjusted(:,1);
                    landmarksScatterBA.YData = XAdjusted(:,2);
                    landmarksScatterBA.ZData = XAdjusted(:,3);
%                     % plot refined estimated trajectory.
%                     set(trajectoryBA, 'XData', smooth(camLocationsAdjusted(:,1)), 'YData', ...
%                     smooth(camLocationsAdjusted(:,2)), 'ZData', smooth(camLocationsAdjusted(:,3)));
%                     camBA.Location = camLocationsAdjusted(end,:);
%                     camBA.Orientation = camOrientationsAdjusted(:,:,end);
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
     figure(f_keypointScores);
        subplot(2,1,1); hold on
            title('Share of Inlier Keypoints');
            plot(frameCount,smooth(scores),'-');
        subplot(2,1,2); 
            title('Number of tracked Keypoints');
            plot(frameCount,smooth(keypointCount),'-'); 
end

fprintf('\n average inlier share: %d \n', mean(scores));