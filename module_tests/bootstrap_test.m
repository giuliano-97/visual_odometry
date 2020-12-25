clear;
close all;

%% Load test data

% Load image pair
img1_file = "./data/parking/images/img_00007.png";
img2_file = "./data/parking/images/img_00008.png";

img1 = rgb2gray(imread(img1_file));
img2 = rgb2gray(imread(img2_file));

% Load camera params
K = load('./data/parking/K.txt');
% Create cameraParameters object using the K matrix
cameraParams = cameraParameters('IntrinsicMatrix', K');

%% Bootstrap
tic
[points, landmarks] = bootstrap(img1, img2, cameraParams, ...
    'PlotResult', true, 'MaxDepth', 2000,...
    'FeatureType', 'Harris',...
    'FeatureMatchingMode', 'KLT', ...
    'MetricThreshold', 700, ...
    'FilterSize', 3, 'MinQuality', 0.001);
toc