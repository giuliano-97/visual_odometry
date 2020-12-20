function [matched_points0, landmarks] = bootstrap(img0, img1, cameraParams)

visualize_scene = true;

% Detect Harris corners in both images
points_0 = detectHarrisFeatures(img0);
points_1 = detectHarrisFeatures(img1);

% Extract neighborhood features
[features0, valid_points0] = extractFeatures(img0, points_0);
[features1, valid_points1] = extractFeatures(img1, points_1);

% Match the features
idx_pairs = matchFeatures(features0, features1);

% Get matched_points in both images
matched_points0 = valid_points0(idx_pairs(:,1), :);
matched_points1 = valid_points1(idx_pairs(:,2), :);

% Estimate the essential matrix
[E, inliers_idx] = estimateEssentialMatrix(...
    matched_points0, matched_points1, cameraParams);

% Set the final set of ouput keypoints for the first view
matched_points0 = matched_points0(inliers_idx, :);
matched_points1 = matched_points1(inliers_idx, :);

% Extract relative camera pose of the second image
[R1, t1] = relativeCameraPose(E, cameraParams,...
    matched_points0, matched_points1);

% Build the camera matrices
M0 = [eye(3); zeros(1,3)];
M1 = [R1; t1];

% Triangulate the landmarks
landmarks = triangulate(matched_points0, matched_points1, M0, M1);

%% (Optionally) Visualize the 3-D scene

if visualize_scene
    
    % Homogenize landmarks coords
    P = [landmarks'; ones(1, size(landmarks,1))];

    figure(1),
    subplot(1,3,1)

    % Plot landmarks in 3D
    plot3(P(1,:), P(2,:), P(3,:), 'o');

    % Display camera poses
    plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

    R_C2_W = R1';
    T_C2_W = t1';
    center_cam2_W = -R_C2_W'*T_C2_W;
    plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

    axis equal
    rotate3d on;
    grid
    
    % Get matched points locations
    p1 = matched_points0.Location';
    p2 = matched_points1.Location';
    
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

