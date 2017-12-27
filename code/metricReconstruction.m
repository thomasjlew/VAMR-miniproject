function [scaledState] = metricReconstruction(state,image)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% Metric reconstruction

actualDistance = 0.2;
scaledState = state;

% detect checkerboard
warning('off','all');
[checkerPoints,boardSize]=detectCheckerboardPoints(image);
warning('on','all');

if ~isempty(checkerPoints)
    % find keypoints and landmarks that coincide with checkerboard points
    [dist,idxNextKeypoints] = pdist2(state.P,cast(checkerPoints,'single'),'euclidean','Smallest',1);
    Xcheckerboard = state.X(idxNextKeypoints,:);
    Pcheckerboard = state.P(idxNextKeypoints,:);
    if ~any(dist>0.2)
        % determine current distance between the outermost corners of the
        % checkerboard
        measuredDistance = norm(Xcheckerboard(1,1:2)-Xcheckerboard(end,1:2));
        pixelDistance = norm(Pcheckerboard(1,:)-Pcheckerboard(end,:))
        
        if pixelDistance>30
            scalingFactor = actualDistance/measuredDistance

            % scale state to fit 
            scaledState.X = state.X*scalingFactor;

            % DEBUG plot
            figure;
            imshow(image); hold on
            scatter(checkerPoints(:,1),checkerPoints(:,2),'b','+');
            scatter(Pcheckerboard([1 9],1),Pcheckerboard([1 9],2),'r','+');
            hold off
        end
    end
end

end

