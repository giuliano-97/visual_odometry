clear;
close all;

%% Load test data

% Load image pair
img1_file = "./data/parking/images/img_00096.png";
img2_file = "./data/parking/images/img_00097.png";

img1 = rgb2gray(imread(img1_file));
img2 = rgb2gray(imread(img2_file));

% Load camera params
K = load('./data/parking/K.txt');
% Create cameraParameters object using the K matrix
cameraParams = cameraParameters('IntrinsicMatrix', K');

%% Bootstrap
tic
[points, landmarks] = bootstrap(img1, img2, cameraParams, ...
    'PlotResult', true, 'MaxDepth', 50);
toc