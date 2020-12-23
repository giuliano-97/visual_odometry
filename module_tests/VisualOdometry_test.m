addpath('../modules');
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

% Test continuous operation
poses = zeros(4,3,10);
poses(:,:,1) = [eye(3); zeros(1,3)];
prev_img = imread(strcat(data_path, sprintf('%06d.png',0)));
for frame_idx=1:9
    curr_img = imread(strcat(data_path, sprintf('%06d.png',frame_idx)));
    
    [state, poses(:,:,frame_idx+1)] = ...
        vo.processFrame(prev_img, curr_img, state);
    
    prev_img = curr_img;
end