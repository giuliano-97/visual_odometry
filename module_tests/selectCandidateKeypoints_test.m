% Load two consecutive images
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

%% Detect and track keypoints

% Detect keypoints in both images
points1 = detectHarrisFeatures(img1);
points2 = detectHarrisFeatures(img2);

% Track keypoints in the second image with KLT
tracker = KLTTracker();
[points1_tracked, validity, ~] = tracker.track(img1, img2, points1.Location);

% Select new non-overlapping keypoints
tic
validIndex = selectCandidateKeypoints(points1_tracked(validity, :), ...
    points2.Location, 'MinDistance', 10);
toc

%% Plot the results
figure(1);
subplot(1,2,1);
img1_marked = insertMarker(img1,points1.Location, '+','Color','white');
imshow(img1_marked);
subplot(1,2,2);
img2_marked = insertMarker(img2, points1_tracked(validity,:),'+','Color','white');
img2_marked = insertMarker(img2_marked, points2.Location(validIndex, :), ...
    'o','Color','red');
imshow(img2_marked);