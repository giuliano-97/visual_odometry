close all;
clear;

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

klt_tracker = KLTTracker('NumPyramidLevels',4,...
                         'MaxBidirectionalError', 2,...
                         'BlockSize', [41, 41],...
                         'MaxIterations', 50);
                     
% Define number of frames to replay and where to start.
num_frames = 200;
start_frame = 1;

% Initializing
data_loader.reset(start_frame);
prev_img = data_loader.next();
if ndims(prev_img)>2
    prev_img = rgb2gray(prev_img);
end
prev_keypoints = detectHarrisFeatures(prev_img, 'MinQuality', 0.0001,...
    'FilterSize', 3).Location;
prev_img_marked = insertMarker(prev_img, prev_keypoints, '+','Color','white');
imshow(prev_img_marked);

for i = start_frame: start_frame+num_frames-1
    curr_img = data_loader.next();
    if ndims(curr_img) > 2
        curr_img = rgb2gray(curr_img);
    end
    [curr_keypoints, valid] = klt_tracker.track(prev_img, curr_img,...
        prev_keypoints);
    curr_img_marked = insertMarker(curr_img, curr_keypoints(valid,:),...
        '+','Color','white');
    imshow(curr_img_marked);
    prev_keypoints = curr_keypoints(valid,:);
    prev_img = curr_img;
end