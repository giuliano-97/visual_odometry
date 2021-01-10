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
    bootstrap_frames = data_loader.bootstrap_frames;
    img0 = data_loader.retrieveFrame(bootstrap_frames(1));
    img1 = data_loader.retrieveFrame(bootstrap_frames(2));
    
    if ndims(img0) == 3
        img0 = rgb2gray(img0);
        img1 = rgb2gray(img1);
    end
    
    [keypoints, landmarks, reproError, pose_bootstrap] = bootstrap(img0, img1, cameraParams, ...
        'MaxNumKeypoints', data_loader.bootstrap_MaxNumKeypoints, ...
        'MinNumLandmarks', data_loader.bootstrap_MinNumLandmarks,...
        'MaxDepth', data_loader.bootstrap_MaxDepth, ...
        'FeatureMatchingMode', data_loader.bootstrap_FeatureMatchingMode, ...
        'FilterSize', data_loader.bootstrap_FilterSize,...
        'MinQuality', data_loader.bootstrap_MinQuality);
    prev_img = data_loader.retrieveFrame(bootstrap_frames(2));
    data_loader.reset(bootstrap_frames(2)+1);
else
    assert(dataset_type==3,...
        'Test without bootstrap only available if using Kitti tutorial dataset');
    keypoints = data_loader.initial_keypoints;
    landmarks = data_loader.initial_landmarks;
    reproError = zeros(size(landmarks,1), 1);
    prev_img = data_loader.retrieveFrame(bootstrap_frames(2));
    pose = [eye(3); zeros(1,3)];
    data_loader.reset(bootstrap_frames(2)+1);
end

% Number of frames to play VO for
num_frames = 35; %data_loader.last_frame - data_loader.index + 1;
assert(num_frames <= data_loader.last_frame-data_loader.index+1,...
    'Not enough frames');

% Initialize the vo pipeline
[H,W] = size(prev_img);
vo = VisualOdometry(cameraParams, [H,W],...
    'MaxTemporalRecall',            data_loader.vo_MaxTemporalRecall, ...
    'MaxNumLandmarks',              data_loader.vo_MaxNumLandmarks, ...
    'MaxReprojectionError',         data_loader.vo_MaxReprojectionError,...
    'angularThreshold',             data_loader.vo_AngularThreshold,...
    'penaltyFactor',                data_loader.vo_PenaltyFactor,...
    'uniformityScoreSigma',         data_loader.vo_UniformityScoreSigma,...
    'KLTnumPyramidLevels',          data_loader.vo_KLTnumPyramidLevels,...
    'KLTmaxBidirectionalError',     data_loader.vo_KLTmaxBidirectionalError,...
    'KLTmaxBidirectionalError',     data_loader.vo_KLTmaxBidirectionalError,...
    'KLTblockSize',                 data_loader.vo_KLTblockSize,...
    'KLTmaxIterations',             data_loader.vo_KLTmaxIterations,...
    'RANSACMaxNumTrials',           data_loader.vo_RANSACMaxNumTrials,...
    'RANSACConfidence',             data_loader.vo_RANSACConfidence,...
    'RANSACMaxReprojectionError',   data_loader.vo_RANSACMAxReprojectionError,...
    'NewCandidateMinQuality',       data_loader.vo_NewCandidateMinQuality,...
    'NewCandidateFilterSize',       data_loader.vo_NewCandidateFilterSize,...
    'NewCandidateMinDistance',      data_loader.vo_NewCandidateMinDistance,...
    'NewCandidateMaxNewKeypoints',  data_loader.vo_NewCandidateMaxNewKeypoints);

% Initialize the state struct
state = initializeState(landmarks, keypoints, reproError);

% Initialize camera poses array
pose = [eye(3); zeros(1,3)];
poses = zeros(4,3,10);
poses(:,:,1) = pose;
poses(:,:,2) = pose_bootstrap;


%% Test continuous operation
% Initialize VO visualizer
vov = VOVisualizer;
vov.update(prev_img, keypoints, [], landmarks, pose_bootstrap);
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

    poses(:,:,i+1-bootstrap_frames(2)) = pose;
    prev_img = curr_img;
end
