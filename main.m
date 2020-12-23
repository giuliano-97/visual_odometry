% Datasets paths
kitti_path = 'data/kitti';
parking_path = 'data/parking';
malaga_path = 'data/malaga';

%% Setup
ds = 2; % 0: KITTI, 1: Malaga, 2: parking
[cameraParams, ground_truth, last_frame] = loadGeneralData(ds);

%% Bootstrap
% Need to set bootstrap_frames
% FIXME: bootstrap frames choice should depend on the dataset
bootstrap_frames = [1,2];
if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

% Bootstrap initial set of keypoints and landmarks
[keypoints, landmarks] = bootstrap(img0, img1, cameraParams);

% Define markovian state
state.landmarks = landmarks;
state.keypoints = keypoints;
state.candidate_keypoints = [];
state.candidate_first_keypoints = [];
state.candidate_first_poses = [];
state.candidate_time_indxs = [];

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
prev_img = img1;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    % Makes sure that plots refresh.    
    pause(0.01);
    
    % Update
    [state, pose] = processFrame(state, prev_img, image);
    prev_img = image;
end
