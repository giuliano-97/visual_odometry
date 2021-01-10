%% Setup environment
close all; clear;
run("setup_env.m");
rng(1023);

%% Load data and metadata
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
if ds == 0
    data_loader = dataLoaderKitti('./data/kitti');
elseif ds == 1
    data_loader = dataLoaderMalaga('./data/malaga-urban-dataset-extract-07');
elseif ds == 2
    data_loader = dataLoaderParking('./data/parking');
else
    assert(false, "Invalid dataset type choose: 0, 1, 2");
end


%% Bootstrap
% Load bootstrap images
bootstrap_frames = data_loader.bootstrap_frames;
img0 = data_loader.retrieveFrame(bootstrap_frames(1));
img1 = data_loader.retrieveFrame(bootstrap_frames(2));

if ndims(img0) == 3
    img0 = rgb2gray(img0);
    img1 = rgb2gray(img1);
end

% Bootstrap keypoints and landmarks
cameraParams = data_loader.camParams;
[keypoints, landmarks, reproError, pose] = bootstrap(img0, img1, ...
    cameraParams, ...
    'MaxNumKeypoints', data_loader.bootstrap_MaxNumKeypoints, ...
    'MinNumLandmarks', data_loader.bootstrap_MinNumLandmarks,...
    'MaxDepth', data_loader.bootstrap_MaxDepth, ...
    'FeatureMatchingMode', data_loader.bootstrap_FeatureMatchingMode, ...
    'FilterSize', data_loader.bootstrap_FilterSize,...
    'MinQuality', data_loader.bootstrap_MinQuality);

% Initialize the vo pipeline
prev_img = img1;
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

% Initialize VO visualizer
vov = VOVisualizer;
vov.update(prev_img, keypoints, [], landmarks, pose);
pause(2.0);

%% Continuous operation

% Initialize empty array of camera pose matrices
pose = [eye(3); zeros(1,3)];
poses = zeros(4,3,10);
poses(:,:,1) = pose;

% Reset data loader
data_loader.reset(bootstrap_frames(2)+1);

% Iterate over all the frames
for i = data_loader.index : data_loader.last_frame
    % Process the next frame
    curr_img = data_loader.next();
    if ndims(curr_img) == 3
        curr_img = rgb2gray(curr_img);
    end
    [state, pose] = ...
        vo.processFrame(prev_img, curr_img, state);
    
    % Refresh the plots
    vov.update(curr_img, state.keypoints, state.candidate_keypoints, ...
        state.landmarks, pose);

    poses(:,:,i+1-bootstrap_frames(2)) = pose;
    prev_img = curr_img;    
end
