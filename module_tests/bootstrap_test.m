clear;
close all;

addpath('../modules');

%% Load test data

% Load image pair
img1_file = "../data/parking/images/img_00128.png";
img2_file = "../data/parking/images/img_00129.png";

img1 = rgb2gray(imread(img1_file));
img2 = rgb2gray(imread(img2_file));

% Load camera params
K = load('../data/parking/K.txt');
% Create cameraParameters object using the K matrix
cameraParams = cameraParameters('IntrinsicMatrix', K');

%% Bootstrap
[points, landmarks] = bootstrap(img1, img2, cameraParams);
