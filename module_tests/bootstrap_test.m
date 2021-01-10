clear;
close all;

%% Load data and metadata
ds = 2; % 0: KITTI, 1: Malaga, 2: parking
if ds == 0
    data_loader = dataLoaderKitti('./data/kitti');
elseif ds == 1
    data_loader = dataLoaderMalaga('./data/malaga-urban-dataset-extract-07');
elseif ds == 2
    data_loader = dataLoaderParking('./data/parking');
else
    assert(false, "Invalid dataset type choose: 0, 1, 2");
end

% Load image pair
bootstrap_frames = data_loader.bootstrap_frames;
img1 = data_loader.retrieveFrame(bootstrap_frames(1));
img2 = data_loader.retrieveFrame(bootstrap_frames(2));

if ndims(img1) == 3
   img1 = rgb2gray(img1);
   img2 = rgb2gray(img2);
end

% Load camera params
cameraParams = data_loader.camParams;

%% Bootstrap
tic
[points, landmarks] = bootstrap(img1, img2, cameraParams, ...
    'PlotResult', true, ...
    'MaxNumKeypoints', data_loader.bootstrap_MaxNumKeypoints, ...
    'MinNumLandmarks', data_loader.bootstrap_MinNumLandmarks,...
    'MaxDepth', data_loader.bootstrap_MaxDepth, ...
    'FeatureMatchingMode', data_loader.bootstrap_FeatureMatchingMode, ...
    'FilterSize', data_loader.bootstrap_FilterSize,...
    'MinQuality', data_loader.bootstrap_MinQuality);
toc