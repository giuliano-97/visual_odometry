close all;
clear;

addpath('../modules');
addpath('../utils');
data_path = "../data/continuous_op_test/";

% Load data
K = load(strcat(data_path, 'K.txt'));
keypoints = load(strcat(data_path, 'keypoints.txt'));
p_W_landmarks = load(strcat(data_path, 'p_W_landmarks.txt'));

% Remove the landmarks which are too close or too far away
validIndex = p_W_landmarks(:,3) > 10 & p_W_landmarks(:,3) < 40;
p_W_landmarks = p_W_landmarks(validIndex,:);
keypoints = keypoints(validIndex,:);
keypoints = [keypoints(:,2), keypoints(:,1)];

% Initialize the vo pipeline
cameraParams = cameraParameters('IntrinsicMatrix', K');
vo = VisualOdometry(cameraParams, 'KeypointsMode', 'KLT');

% Initialize the state struct
state.landmarks = p_W_landmarks;
state.keypoints = keypoints;

% Initialize camera poses array
pose = [eye(3); zeros(1,3)];
poses = zeros(4,3,10);
poses(:,:,1) = pose;

% Plot the 3D landmarks and the first camera
figure(1);
subplot(2,1,1);
plot3(p_W_landmarks(:,1), p_W_landmarks(:,2), p_W_landmarks(:,3), '*');
hold on;
plotCameraPose(pose, 'Camera 1');
axis equal;
grid on;

% Test continuous operation
prev_img = imread(strcat(data_path, sprintf('%06d.png',0)));
% Plot initial set of keypoints
subplot(2,1,2);
imshow(insertMarker(prev_img, keypoints));
for frame_idx=2:9
    curr_img = imread(strcat(data_path, sprintf('%06d.png',frame_idx)));
    
    [state, pose] = ...
        vo.processFrame(prev_img, curr_img, state);
    
    subplot(2,1,1);
    plotCameraPose(pose, sprintf('Camera %d', frame_idx));
    
    poses(:,:,frame_idx+1) = pose;
    
    prev_img = curr_img;
end