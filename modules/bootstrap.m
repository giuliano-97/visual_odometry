function [keypoints, landmarks, pose] = bootstrap(img0, img1, cameraParams,...
    optionalArgs)
    arguments
        img0 
        img1
        cameraParams
        optionalArgs.PlotResult logical = false
        optionalArgs.MaxDepth double = 1000;
        optionalArgs.MinNumLandmarks uint32 = 12;
        optionalArgs.FeatureMatchingType uint8 = 1;     % 0: matlabMatching 1: KLT matching
    end
    
%% Extract bootstrap set of keypoints and landmarks

% Detect Harris corners in both images
points_0 = detectHarrisFeatures(img0, 'FilterSize', 3);

if optionalArgs.FeatureMatchingType == 0
    points_1 = detectHarrisFeatures(img1);
    % Extract neighborhood features
    [features0, valid_points0] = extractFeatures(img0, points_0);
    [features1, valid_points1] = extractFeatures(img1, points_1);

    % Match the features
    idx_pairs = matchFeatures(features0, features1);

    % Get matched_points in both images
    matched_points0 = valid_points0(idx_pairs(:,1), :).Location;
    matched_points1 = valid_points1(idx_pairs(:,2), :).Location;
else
    klt_tracker = KLTTracker();
    [points, validity, ~] = klt_tracker.track(img0, img1, points_0.Location);
    matched_points0 = points_0(validity, :).Location;
    matched_points1 = points(validity, :);
end

% Due to the implicit randomness of this procedure which may cause no
% landmark to be triangulated, we repeat until a number of landmarks
% larger or equal than the minimum required is found
ok = false;
while ~ok
    % Estimate the essential matrix
    [E, inliers_idx] = estimateEssentialMatrix(...
        matched_points0, matched_points1, cameraParams);

    
    % Set the final set of ouput keypoints for the first view
    matched_points0_inliers = matched_points0(inliers_idx, :);
    matched_points1_inliers = matched_points1(inliers_idx, :);

   % Extract relative camera pose of the second image
    [R1, t1, validPointsFraction] = relativeCameraPose(E, cameraParams,...
        matched_points0_inliers, matched_points1_inliers);
    
    % Check valid point fractions and if R1 or t1 contain nans
    if validPointsFraction < 0.9 || sum(isnan(R1(:))) > 0 || ...
            sum(isnan(t1)) > 0
        continue;
    end

    % Build the camera matrices
    [orientation0, position0] = cameraPoseToExtrinsics(eye(3), zeros(1,3));
    [orientation1, position1] = cameraPoseToExtrinsics(R1, t1);
    M0 = cameraMatrix(cameraParams, orientation0, position0);
    M1 = cameraMatrix(cameraParams, orientation1, position1);

    % Triangulate the landmarks
    [landmarks, ~, validIndex] = triangulate(matched_points0_inliers,...
        matched_points1_inliers, M0, M1);

    % Remove points which are also too far away
    validIndex = validIndex & landmarks(:,3) < optionalArgs.MaxDepth;
    
    ok = nnz(validIndex) >= optionalArgs.MinNumLandmarks;
end

pose = [R1; t1];
% Keep only valid points
landmarks = landmarks(validIndex, :);
keypoints = matched_points1(validIndex, :);

%% (Optionally) Visualize the 3-D scene

if optionalArgs.PlotResult == true

    figure(1);
    subplot(1,3,1);
   
    % Plot landmarks in 3D
    plot3(landmarks(:,1), landmarks(:,2), landmarks(:,3),'*');
    hold on;
    
    % Convert to rigid body transform object
    absPose0 = rigid3d(eye(3), zeros(1,3));
    absPose1 = rigid3d(R1, t1);
    % Plot the cameras
    plotCamera('AbsolutePose', absPose0, 'Size', 1);
    text(0,0,0,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
    plotCamera('AbsolutePose', absPose1, 'Size', 1);
    text(absPose1.Translation(1), absPose1.Translation(2), absPose1.Translation(3), ...
        'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    
    set(gca,'CameraUpVector',[0 1 0]);
    axis equal
    grid
    
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    
    % Get matched points locations
    p1 = matched_points0(validIndex, :)';
    p2 = matched_points1(validIndex, :)';
    
    % Display matched points
    subplot(1,3,2)
    imshow(img0,[]);
    hold on
    plot(p1(1,:), p1(2,:), 'ys');
    title('Image 1')

    subplot(1,3,3)
    imshow(img1,[]);
    hold on
    plot(p2(1,:), p2(2,:), 'ys');
    title('Image 2')

end

