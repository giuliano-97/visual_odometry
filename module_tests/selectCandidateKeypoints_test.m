% Load two consecutive images
clear;
close all;

%% Load test data

% Load image pair
img1_file = "data/parking/images/img_00128.png";
img2_file = "data/parking/images/img_00129.png";

img1 = rgb2gray(imread(img1_file));
img2 = rgb2gray(imread(img2_file));

% Load camera params
K = load('data/parking/K.txt');
% Create cameraParameters object using the K matrix
cameraParams = cameraParameters('IntrinsicMatrix', K');

%% Detect and track keypoints

% Detect keypoints in both images
% points = detectHarrisFeatures(img1, 'MinQuality', 0.0008, 'FilterSize', 3);
% points = detectFASTFeatures(img1, 'MinQuality', 0.001);
points = detectMinEigenFeatures(img1, 'MinQuality', 0.005);

% Track keypoints in the second image with KLT
tracker = KLTTracker();
[points_tracked, validity, ~] = tracker.track(img1, img2, points.Location);

% Select new non-overlapping keypoints
new_points = selectCandidateKeypoints(img2, points_tracked(validity,:), ...
    'MinDistance', 5, 'MinQuality', 0.0005, 'MaxNewKeypoints', 100, ...,
    'FilterSize', 5, 'FractionToKeep', 1.0);

%% Plot the results
figure(1);
subplot(1,2,1);
imshow(insertMarker(img1,points.Location, '+','Color','blue'));
subplot(1,2,2);
img2_marked = insertMarker(img2, points_tracked(validity,:),'+','Color','blue');
img2_marked = insertMarker(img2_marked, new_points, ...
    '*','Color','red');
imshow(img2_marked);