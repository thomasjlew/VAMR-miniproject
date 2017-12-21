function cameraParams = calibrateDuckiebotCamera ()
    % Path to the files for the dataset
    dataset_path = '';
    
    % Parameters
    debug = true;
    square_size = 30; % in units of 'mm' i.e 30mm

    % Define images to process for calibration
    imageFileNames = {strcat(dataset_path, ), % and so on
    };
    
    % Detect the checkerboards in different images using the Matlab
    % function
    [imagePoints, board_size, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    
    % Generate world coordinates of the corners of the squares
    world_points = generateCheckerboardPoints(board_size, square_size);

    % Calibrate the camera
    [cameraParams, ~, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
        'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);
    
    if (debug)
        % View reprojection errors
        figure; showReprojectionErrors(cameraParams, 'BarGraph');
        % Visualize pattern locations
        figure; showExtrinsics(cameraParams, 'CameraCentric');
        % Display parameter estimation errors
        displayErrors(estimationErrors, cameraParams);
    end
end
