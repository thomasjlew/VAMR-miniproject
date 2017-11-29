function plot_Trajectory_w_Landmarks(P,R,T, nfig)


% Display the camera pose

figure(nfig)
% The first coordinate frame is centered at origin

plotCoordinateFrame(eye(3), zeros(3,1), 10); % No rotation

% The second coordinate frame is rotated and translated with known matrices

% The center of the next camera is defined with rotation/translation
center_transl = -R'*T;
plotCoordinateFrame(R', center_transl,10);

% Landmarks

hold on;
scatter3(P(1,:), P(2,:), P(3,:), '.')
view([0 -1 0])
hold off;
end
