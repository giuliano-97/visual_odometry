close all;
clear;

% Fix random seed for reproducibility
rng(1023);

% Load data
test_bootstrap = true;

dataset_type = 0; % 0: KITTI, 1: malaga, 2: parking, 3:KITTI_tutorial

% Pick the correspoinding data loader
if dataset_type ==0
    data_loader = dataLoaderKitti('./data/kitti');
elseif dataset_type == 1
    data_loader = dataLoaderMalaga('./data/malaga-urban-dataset-extract-07');
elseif dataset_type == 2
    data_loader = dataLoaderParking('./data/parking');
elseif dataset_type == 3
    data_loader = dataLoaderKittiTutorial('./data/continuous_op_test');
else
    assert(false, "Invalid dataset type choose: 0, 1,2,3");
end

cameraParams = data_loader.camParams;

if test_bootstrap
    % Load bootstrap images
    bootstrap_frames = [0,1];
    img0 = data_loader.retrieveFrame(bootstrap_frames(1));
    img1 = data_loader.retrieveFrame(bootstrap_frames(2));
    
    if ndims(img0) == 3
        img0 = rgb2gray(img0);
        img1 = rgb2gray(img1);
    end
    [keypoints, landmarks, pose] = bootstrap(img0, img1, cameraParams, ...
        'PlotResult', true, 'MinNumLandmarks', 300,...
        'MaxDepth', 300, ...
        'FeatureMatchingMode', 'KLT', ...
        'FilterSize', 3, 'MinQuality', 0.001);
    prev_img = data_loader.retrieveFrame(bootstrap_frames(2));
    data_loader.reset(bootstrap_frames(2)+1);
else
    assert(dataset_type==3,...
        'Test without bootstrap only available if using Kitti tutorial dataset');
    keypoints = data_loader.initial_keypoints;
    landmarks = data_loader.initial_landmarks;
    prev_img = data_loader.retrieveFrame(bootstrap_frames(2));
    pose = [eye(3); zeros(1,3)];
    data_loader.reset(bootstrap_frames(2)+1);
end
% Initialize the vo pipeline
vo = VisualOdometry(cameraParams);

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
% Number of frames to play VO for
num_frames = 50;
assert(num_frames <= data_loader.last_frame-data_loader.index+1,...
    'Not enougth frames');

if ndims(prev_img) == 3
    prev_img = rgb2gray(prev_img);
end

for i = data_loader.index : data_loader.index+num_frames-1
    curr_img = data_loader.next();
    if ndims(curr_img) == 3
        curr_img = rgb2gray(curr_img);
    end
    
    [state, pose] = ...
        vo.processFrame(prev_img, curr_img, state);
    
    subplot(2,2,[1,2]);
    plot3(state.landmarks(:,1), state.landmarks(:,2), state.landmarks(:,3), '*');
    
    % Plot camera pose
    subplot(2,2,[1,2]);
    plotCameraPose(pose, sprintf('Camera %d', i));
    
    pause(0.01);    hold on;

    
    % Update keypoints view
    subplot(2,2,[3,4]);
    imshow(insertMarker(curr_img, state.keypoints, 'o', 'Color', 'red'));
    
    
    poses(:,:,i+1) = pose;
    prev_img = curr_img;
    state
end