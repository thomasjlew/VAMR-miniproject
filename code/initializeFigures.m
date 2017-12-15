function [f_trackingP,f_keypointScores,f_cameraTrajectory,cam,trajectory,landmarks_scatter] = initializeFigures(location_initial,orientation_inital,X_initial)
%INITIALIZEFIGURES Summary of this function goes here
%   Detailed explanation goes here
%% Intialize plots
% plot of inlier and outlier keypoints on current images
f_trackingP = figure('Name','Feature Tracking Keypoints');
    set(gcf, 'Position', [800, 1000, 500, 500])
% plot of RANSAC inlier share among all tracked keypoints
f_keypointScores = figure('Name','Share of Inliers');
    set(gcf, 'Position', [800, 200, 500, 150])
    subplot(1,2,1);
        xlabel('frame count');ylabel('share of inliers');
        title('Share of Inlier Keypoints');
    subplot(1,2,2);
        xlabel('frame count');ylabel('number keypoint');
        title('Number of tracked Keypoints');
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
    plotCamera('Size', cameraSize, 'Location',...
        [0 0 0], 'Orientation', eye(3),'Color', 'k', 'Opacity', 0.5);
    cam = plotCamera('Size', cameraSize, 'Location',location_initial, ...
        'Label','Current Pose', 'Orientation', orientation_inital,'Color', 'r', 'Opacity', 0.5);
    trajectory = plot3(0, 0, 0, 'r-','LineWidth',3);   
    title('Camera trajectory');
    % plot 3D landmarks
    landmarks_scatter = scatter3(X_initial(:, 1), X_initial(:, 2), X_initial(:, 3), 8, 'o','b'); grid on;
    legend('Estimated Trajectory');
    legend('AutoUpdate','off');

end

