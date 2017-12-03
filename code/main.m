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
    'MinQuality', 1e-5, ...             % Harris detection parameters
    'FilterSize', 3, ...
    'MaxRatio', 0.99,...%0.8, ...   % bigger => more matches
    'MatchThreshold', 100.0,...      % lower  => less
    'BlockSizeHarris', 11, ...                % Feature extraction parameters
    'Unique', false,...%true,...
    'BlockSize',[21 21], ...            % KLT parameters
    'MaxIterations',30, ...         
    'NumPyramidLevels',3, ...
    'MaxBidirectionalError',1,...%inf, ... %%% REMOVES POINTS WHEN NOT TRACKED ANYMORE (vision.PointTracker)   
    'MaxNumTrials',3000, ...            % P3P and RANSAC parameters 
    'Confidence',70,...%50, ...
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
scatter(t_initial(1),t_initial(2),t_initial(3),'g+');
legend('3d pts', '1st cam', '2nd cam');

% Save camera poses
cam_poses = zeros(3,4,2);
cam_poses(:,:,1) = [eye(3,3),[0;0;0]];
cam_poses(:,:,2) = [R_initial,t_initial'];

%% Continuous operation
NB_FRAMES_LOOP = 100;
% only iterate over 9 images for testing purposes
range = (bootstrap_frames(2)+1):bootstrap_frames(2)+NB_FRAMES_LOOP;
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
    
    % Process next frame and update camera poses
    [state,pose] = processFrame(prev_state,prev_img,image,params_continouos,K,figure_KLT);
    cam_poses(:,:,size(cam_poses,3)+1) = pose;
    last_R = cam_poses(1:3,1:3,end); last_t = cam_poses(1:3,4,end);
    
    
    figure(f_cameraPose);
%     scatter3(X_initial(:, 1), X_initial(:, 2), X_initial(:, 3), 5); hold on; grid on;
%     plotCamera('Location', last_t', 'Orientation', last_R, 'Opacity', 0, 'Color', [0,1,0]);
    p1=last_t;
    p2=last_t+last_R*[0;0;3];
    mArrow3(p1,p2, 'stemWidth', 0.05);
%     scatter3(last_t(1),last_t(2),last_t(3),'b+');
%     legend('3d pts', '1st cam', '2nd cam');

    % Plot camera pose and landmarks
    %figure(f_cameraPose);
    %subplot(1,3,3);
    %TODO
    %R = pose(:,1:3);
    %t = pose(:,4);
    %plotCoordinateFrame(R', t, 0.7);
    %view(0,0);
    
    % Makes sure that plots refresh.  
    disp('  ');
    disp('-> Plotting');
%     autoArrangeFigures(0,0,1); % Don't hesitate to remove if you want
    drawnow;
%     pause(0.01);
    
    % Update input varibles for next iteration
    prev_img = image;
    prev_state = state;
    
    % DEBUG
    disp('State: 3D pts nb: ' + string(length(state.X)));
    
end









%% Additionnal plotting functions
function h = mArrow3(p1,p2,varargin)
    %mArrow3 - plot a 3D arrow as patch object (cylinder+cone)
    %
    % syntax:   h = mArrow3(p1,p2)
    %           h = mArrow3(p1,p2,'propertyName',propertyValue,...)
    %
    % with:     p1:         starting point
    %           p2:         end point
    %           properties: 'color':      color according to MATLAB specification
    %                                     (see MATLAB help item 'ColorSpec')
    %                       'stemWidth':  width of the line
    %                       'tipWidth':   width of the cone                       
    %
    %           Additionally, you can specify any patch object properties. (For
    %           example, you can make the arrow semitransparent by using
    %           'facealpha'.)
    %                       
    % example1: h = mArrow3([0 0 0],[1 1 1])
    %           (Draws an arrow from [0 0 0] to [1 1 1] with default properties.)
    %
    % example2: h = mArrow3([0 0 0],[1 1 1],'color','red','stemWidth',0.02,'facealpha',0.5)
    %           (Draws a red semitransparent arrow with a stem width of 0.02 units.)
    %
    % hint:     use light to achieve 3D impression
    %

    propertyNames = {'edgeColor'};
    propertyValues = {'none'};    

    %% evaluate property specifications
    for argno = 1:2:nargin-2
        switch varargin{argno}
            case 'color'
                propertyNames = {propertyNames{:},'facecolor'};
                propertyValues = {propertyValues{:},varargin{argno+1}};
            case 'stemWidth'
                if isreal(varargin{argno+1})
                    stemWidth = varargin{argno+1};
                else
                    warning('mArrow3:stemWidth','stemWidth must be a real number');
                end
            case 'tipWidth'
                if isreal(varargin{argno+1})
                    tipWidth = varargin{argno+1};
                else
                    warning('mArrow3:tipWidth','tipWidth must be a real number');
                end
            otherwise
                propertyNames = {propertyNames{:},varargin{argno}};
                propertyValues = {propertyValues{:},varargin{argno+1}};
        end
    end            

    %% default parameters
    if ~exist('stemWidth','var')
        ax = axis;
        if numel(ax)==4
            stemWidth = norm(ax([2 4])-ax([1 3]))/300;
        elseif numel(ax)==6
            stemWidth = norm(ax([2 4 6])-ax([1 3 5]))/300;
        end
    end
    if ~exist('tipWidth','var')
        tipWidth = 3*stemWidth;
    end
    tipAngle = 22.5/180*pi;
    tipLength = tipWidth/tan(tipAngle/2);
    ppsc = 50;  % (points per small circle)
    ppbc = 250; % (points per big circle)

    %% ensure column vectors
    p1 = p1(:);
    p2 = p2(:);

    %% basic lengths and vectors
    x = (p2-p1)/norm(p2-p1); % (unit vector in arrow direction)
    y = cross(x,[0;0;1]);    % (y and z are unit vectors orthogonal to arrow)
    if norm(y)<0.1
        y = cross(x,[0;1;0]);
    end
    y = y/norm(y);
    z = cross(x,y);
    z = z/norm(z);

    %% basic angles
    theta = 0:2*pi/ppsc:2*pi; % (list of angles from 0 to 2*pi for small circle)
    sintheta = sin(theta);
    costheta = cos(theta);
    upsilon = 0:2*pi/ppbc:2*pi; % (list of angles from 0 to 2*pi for big circle)
    sinupsilon = sin(upsilon);
    cosupsilon = cos(upsilon);

    %% initialize face matrix
    f = NaN([ppsc+ppbc+2 ppbc+1]);

    %% normal arrow
    if norm(p2-p1)>tipLength
        % vertices of the first stem circle
        for idx = 1:ppsc+1
            v(idx,:) = p1 + stemWidth*(sintheta(idx)*y + costheta(idx)*z);
        end
        % vertices of the second stem circle
        p3 = p2-tipLength*x;
        for idx = 1:ppsc+1
            v(ppsc+1+idx,:) = p3 + stemWidth*(sintheta(idx)*y + costheta(idx)*z);
        end
        % vertices of the tip circle
        for idx = 1:ppbc+1
            v(2*ppsc+2+idx,:) = p3 + tipWidth*(sinupsilon(idx)*y + cosupsilon(idx)*z);
        end
        % vertex of the tiptip
        v(2*ppsc+ppbc+4,:) = p2;

        % face of the stem circle
        f(1,1:ppsc+1) = 1:ppsc+1;
        % faces of the stem cylinder
        for idx = 1:ppsc
            f(1+idx,1:4) = [idx idx+1 ppsc+1+idx+1 ppsc+1+idx];
        end
        % face of the tip circle
        f(ppsc+2,:) = 2*ppsc+3:(2*ppsc+3)+ppbc;
        % faces of the tip cone
        for idx = 1:ppbc
            f(ppsc+2+idx,1:3) = [2*ppsc+2+idx 2*ppsc+2+idx+1 2*ppsc+ppbc+4];
        end

    %% only cone v
    else
        tipWidth = 2*sin(tipAngle/2)*norm(p2-p1);
        % vertices of the tip circle
        for idx = 1:ppbc+1
            v(idx,:) = p1 + tipWidth*(sinupsilon(idx)*y + cosupsilon(idx)*z);
        end
        % vertex of the tiptip
        v(ppbc+2,:) = p2;
        % face of the tip circle
        f(1,:) = 1:ppbc+1;
        % faces of the tip cone
        for idx = 1:ppbc
            f(1+idx,1:3) = [idx idx+1 ppbc+2];
        end
    end

    %% draw
    fv.faces = f;
    fv.vertices = v;
    h = patch(fv);
    for propno = 1:numel(propertyNames)
        try
            set(h,propertyNames{propno},propertyValues{propno});
        catch
            disp(lasterr)
        end
    end
end
