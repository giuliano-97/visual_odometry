close all;
clear;

% Fix random seed for reproducibility
rng(1023);

% Load data
test_bootstrap = true;
if test_bootstrap
    [cameraParams, ~, ~] = loadGeneralData(0); % loadGeneralData(2);
    data_path = "data/kitti/00/image_0/"; %"data/parking/";
    image_path_template = strcat(data_path, "%06d.png"); % strcat(data_path, "images/img_%05d.png");
    % Load bootstrap images
    bootstrap_frames = [0,1];
    img0 = imread(sprintf(image_path_template, ...
        bootstrap_frames(1)));
    img1 = imread(sprintf(image_path_template, ...
        bootstrap_frames(2)));
    if ndims(img0) == 3
        img0 = rgb2gray(img0);
        img1 = rgb2gray(img1);
    end
    [keypoints, landmarks, pose] = bootstrap(img0, img1, cameraParams, ...
        'PlotResult', true, 'MinNumLandmarks', 300,...
        'MaxDepth', 300, ...
        'FeatureMatchingMode', 'KLT', ...
        'FilterSize', 3, 'MinQuality', 0.001);
    prev_img = imread(sprintf(image_path_template, ...
        bootstrap_frames(2)));
    next_idx = bootstrap_frames(2) + 1;
else
    data_path = "./data/continuous_op_test/";
    image_path_template = strcat(data_path, "%06d.png");
    K = load(strcat(data_path, 'K.txt'));
    cameraParams = cameraParameters('IntrinsicMatrix', K');
    keypoints = load(strcat(data_path, 'keypoints.txt'));
    keypoints = [keypoints(:,2), keypoints(:,1)];
    landmarks = load(strcat(data_path, 'p_W_landmarks.txt'));
    prev_img = imread(sprintf(image_path_template,0));
    pose = [eye(3); zeros(1,3)];
    next_idx = 1;
end
% Initialize the vo pipeline
vo = VisualOdometry(cameraParams, 'KeypointsMode', 'KLT');

% Initialize the state struct
state = initializeState(landmarks, keypoints, pose, 50);

% Initialize camera poses array
pose = [eye(3); zeros(1,3)];
poses = zeros(4,3,10);
poses(:,:,1) = pose;

% Plot the 3D landmarks and the first camera
figure(1);
subplot(2,2,[1,2]);
plot3(landmarks(:,1), landmarks(:,2), landmarks(:,3), 'o');
hold on;
plotCameraPose(pose, 'Camera 0');
set(gca,'CameraUpVector',[0 1 0]);
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;

pause(3);

%% Test continuous operation
% Plot initial set of keypoints
subplot(2,2,[3,4]);
imshow(insertMarker(prev_img, keypoints, 'o', 'Color', 'red'));
pause(0.5);
for i=0:7
    frame_idx = next_idx + i;
    curr_img = imread(sprintf(image_path_template,frame_idx));
    if ndims(curr_img) == 3
        curr_img = rgb2gray(curr_img);
    end
    
    [state, pose] = ...
        vo.processFrame(prev_img, curr_img, state);
    
    subplot(2,2,[1,2]);
    plot3(state.landmarks(:,1), state.landmarks(:,2), state.landmarks(:,3), '*');
    
    % Plot camera pose
    subplot(2,2,[1,2]);
    plotCameraPose(pose, sprintf('Camera %d', frame_idx));
    
    pause(0.01);    hold on;

    
    % Update keypoints view
    subplot(2,2,[3,4]);
    imshow(insertMarker(curr_img, state.keypoints, 'o', 'Color', 'red'));
    
    
    poses(:,:,frame_idx+1) = pose;
    prev_img = curr_img;
    state
end