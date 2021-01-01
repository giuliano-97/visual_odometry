function [keypoints, landmarks, reproError, pose] = bootstrap(img0, img1, cameraParams,...
    optionalArgs)
%% BOOTSTRAP Creates initial set of 2D/3D correspondences
%   [keypoints, landmarks, pose] = bootstrap(img0, img1, cameraParams)
%       Given two views and the corresponding camera parameters, this
%       function returns a set of 2D/3D correspondences required to
%       bootstrap a monocular VO pipeline.
    arguments
        img0    % The first view
        img1    % The second view
        cameraParams    % The camera parameters
        optionalArgs.PlotResult logical = false
        optionalArgs.MinDepth double = 0    % Min depth of the triangulated landmarks
        optionalArgs.MaxDepth double = 5000 % Max depth of the triangulated landmarks
        optionalArgs.MinNumLandmarks uint32 = 50 % Min number of triangulated landmarks
        optionalArgs.FeatureMatchingMode string ...
            {mustBeMember(optionalArgs.FeatureMatchingMode,...
            {'KLT', 'HardMatching'})} = 'KLT'
        optionalArgs.MinQuality double = 0.01 % Min. quality of detected Harris features 
        optionalArgs.FilterSize double = 5 % Filter size for Harris feature detection
        optionalArgs.uniformFeaturesPercentage double = 100 % Number of uniform features picked
    end
    
%% Extract bootstrap set of keypoints and landmarks

% Detect Harris features in the first view
% points_0 = detectHarrisFeatures(img0,...
%     'MinQuality', optionalArgs.MinQuality,...
%     'FilterSize', optionalArgs.FilterSize);
points_0 = detectMinEigenFeatures(img0,...
    'MinQuality', optionalArgs.MinQuality,...
    'FilterSize', optionalArgs.FilterSize);

% Select points uniformly distributed
% FIXME: this is hard-coded - not good!
points_0 = selectUniform(points_0,...
    round(length(points_0)*optionalArgs.uniformFeaturesPercentage/100),...
    size(img0));

if strcmp(optionalArgs.FeatureMatchingMode, 'HardMatching')
    % Detect keypoint features in the second image
    points_1 = detectHarrisFeatures(img1,...
        'MinQuality', optionalArgs.MinQuality,...
        'FilterSize', optionalArgs.FilterSize);

    % Extract neighborhood features
    [features0, valid_points0] = extractFeatures(img0, points_0);
    [features1, valid_points1] = extractFeatures(img1, points_1);

    % Match the features
    idx_pairs = matchFeatures(features0, features1, 'Unique', true);

    % Get matched_points in both images
    matched_points0 = valid_points0(idx_pairs(:,1));
    matched_points1 = valid_points1(idx_pairs(:,2));
else
    klt_tracker = KLTTracker();
    [points, validity, ~] = klt_tracker.track(img0, img1, points_0.Location);
    matched_points0 = points_0(validity);
    matched_points1 = cornerPoints(points(validity, :));
end

% Build camera intrinsics object - required for triangulation
K = cameraParams.IntrinsicMatrix;
focalLength = [K(1,1), K(2,2)];
principalPoint = [K(3,1), K(3,2)];
imageSize = size(img0);
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Due to the implicit randomness of this procedure which may cause no
% landmark to be triangulated, we repeat until a number of landmarks
% larger or equal than the minimum required is found
ok = false;
while ~ok
    % Estimate the essential matrix
    [E, inliers_idx] = estimateEssentialMatrix(...
        matched_points0.Location, matched_points1.Location, ...
        cameraParams, 'MaxNumTrials', 2000, 'Confidence', 99.5, ...
        'MaxDistance', 0.09);
    
    % If the inliers set is smaller than the min num of landmarks required,
    % repeat the essential matrix estimation
    num_inliers = nnz(inliers_idx);
    if num_inliers < optionalArgs.MinNumLandmarks
        continue;
    end

    % Set the final set of ouput keypoints for the first view
    matched_points0_inliers = matched_points0(inliers_idx);
    matched_points1_inliers = matched_points1(inliers_idx);

   % Extract relative camera pose of the second view
    [R1, t1, validPointsFraction] = relativeCameraPose(E, cameraParams,...
        matched_points0_inliers.Location, matched_points1_inliers.Location);
    
    % Check valid point fractions and if R1 or t1 contain nans
    if num_inliers * validPointsFraction < optionalArgs.MinNumLandmarks ...
            || sum(isnan(R1(:))) > 0 || sum(isnan(t1)) > 0
        continue;
    end
    
    % Add views with keypoints to viewset
    vSet = imageviewset;
    vSet = addView(vSet, 1, rigid3d(eye(3), zeros(1,3)), 'Points', ...
        matched_points0_inliers);
    vSet = addView(vSet, 2, rigid3d(R1, t1), 'Points',...
        matched_points1_inliers);
    
    % Add correspondences to viewset
    vSet = addConnection(vSet, 1, 2, 'Matches', ...
        [1:num_inliers; 1:num_inliers]');
    
    % Find tracks
    tracks = findTracks(vSet);
    
    % Get camera poses
    cameraPoses = poses(vSet);
    
    % Triangulate points
    [landmarks, reproError, validIndex] = triangulateMultiview(tracks, ...
        cameraPoses, intrinsics);
    
    % Keep points in the required view frustum
    validIndex = validIndex & landmarks(:,3) > optionalArgs.MinDepth ...
        & landmarks(:,3) <= optionalArgs.MaxDepth;
    
    % Check if valid points are enough
    ok = nnz(validIndex) >= optionalArgs.MinNumLandmarks;
end

% Keep only valid stuff
pose = [R1; t1];
landmarks = landmarks(validIndex, :);
keypoints = double(matched_points1_inliers(validIndex).Location);
reproError = reproError(validIndex, :);

%% (Optionally) Visualize the 3-D scene

if optionalArgs.PlotResult == true

    figure(1);
    subplot(2,2,[1,2]);
   
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
    
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    % Display matched points
    subplot(2,2,3)
    imshow(insertMarker(img0, matched_points0_inliers(validIndex),...
        '*', 'Color', 'red'));
    title('Image 1')

    subplot(2,2,4)
    imshow(insertMarker(img1, matched_points1_inliers(validIndex),...
        '*', 'Color', 'red'));
    title('Image 2')

end

