function plotCameraPoseLandmark(img, struct, inlier_mask, nnz_inlier_mask...
    R_C_W, t_C_W, n_fig, laufvariable, cam_center1, cam_centera_all)

% X = 3D-landmark [3xN] vom struct 

% P = 2D- keypoints [2xN]

% P_obs = last observed keypoints
% R_C_W rotation matrix [3x3]

figure(n_fig);
imshow(img);
title('Image with Landmarks');
%plot keypoints 2D
hold on;
plot(X(2,:),X(1,:),'bx','Linewidth', 2);
hold off; 


%% 3D
% plot 3-D frame
% plot3(X1,Y1,Z1,LineSpec,...)cam_center1_all(1,:)' ...


% plot the key points 3D
hold on;
scatter3(X(1,:),X(2,:), X(3,:), '.');
% view
az = 45;
el = 45;
view(az,el);
hold off;
