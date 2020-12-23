addpath('../modules');
addpath('../utils');
data_path = "../data/continuous_op_test/";

% Load data
K = load(strcat(data_path, 'K.txt'));
keypoints = load(strcat(data_path, 'keypoints.txt'));
p_W_landmarks = load(strcat(data_path, 'p_W_landmarks.txt'));

% Initialize the vo pipeline
cameraParams = cameraParameters('IntrinsicMatrix', K');
vo = VisualOdometry(cameraParams);

% Initialize the state struct
state.landmarks = p_W_landmarks;
state.keypoints = keypoints;

% Initialize camera poses array
pose = [eye(3); zeros(1,3)];
poses = zeros(4,3,10);
poses(:,:,1) = pose;

% Plot the 3D landmarks and the first camera
figure(1);
subplot(1,3,1);
plot3(p_W_landmarks(:,1), p_W_landmarks(:,2), p_W_landmarks(:,3), '*');
hold on;
plotCameraPose(pose, 'Camera 0');
axis equal;
grid on;

% Test continuous operation
prev_img = imread(strcat(data_path, sprintf('%06d.png',0)));
for frame_idx=1:9
    curr_img = imread(strcat(data_path, sprintf('%06d.png',frame_idx)));
    
    [state, pose] = ...
        vo.processFrame(prev_img, curr_img, state);
    
    plotCameraPose(pose, sprintf('Camera %d', frame_idx));
    
    poses(:,:,frame_idx+1) = pose;
    
    prev_img = curr_img;
end