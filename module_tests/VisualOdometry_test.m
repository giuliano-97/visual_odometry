close all;
clear;

% Fix random seed for reproducibility
rng(1023);

% Load data
test_bootstrap = true;

dataset_type = 2; % 0: KITTI, 1: malaga, 2: parking, 3:KITTI_tutorial

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
        'MinNumLandmarks', 200,...
        'MaxDepth', 200, ...
        'FeatureMatchingMode', 'KLT', ...
        'FilterSize', 5, 'MinQuality', 0.001);
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

% Number of frames to play VO for
num_frames = 300;
assert(num_frames <= data_loader.last_frame-data_loader.index+1,...
    'Not enough frames');

% Initialize the vo pipeline
max_temporal_recall = 20;
vo = VisualOdometry(cameraParams, ...
    'MaxTemporalRecall', max_temporal_recall, ...
    'MaxNumLandmarks', 800, ...
    'MaxReprojectionError', 4);

% Initialize the state struct
state = initializeState(landmarks, keypoints, pose, max_temporal_recall);

% Initialize camera poses array
pose = [eye(3); zeros(1,3)];
poses = zeros(4,3,10);
poses(:,:,1) = pose;


%% Test continuous operation
% Initialize VO visualizer
vov = VOVisualizer;
vov.update(prev_img, keypoints, [], landmarks, pose);
pause(2.5);


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
    
    % Update the visualization
    vov.update(curr_img, state.keypoints, state.candidate_keypoints, ...
        state.landmarks, pose);

    poses(:,:,i+1) = pose;
    prev_img = curr_img;
end
