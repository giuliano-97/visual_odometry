close all
addpath("../modules")
img1_file = "../data/parking/images/img_00200.png";
img2_file = "../data/parking/images/img_00201.png";

img1 = imread(img1_file);
img2 = imread(img2_file);
img1_gray = rgb2gray(img1);
img2_gray = rgb2gray(img2);


klt_tracker = KLTTracker('NumPyramidLevels',3,...
                         'MaxBidirectionalError', 1,...
                         'BlockSize', [31, 31],...
                         'MaxIterations', 30);
                     
tic
img1_pts = detectHarrisFeatures(img1_gray, 'MinQuality', 0.01);
img1_marked = insertMarker(img1,img1_pts.Location, '+','Color','white');
[new_points, validity, scores] = klt_tracker.track(img1_gray, img2_gray, img1_pts.Location);
img2_marked = insertMarker(img2, new_points(validity,:),'+','Color', 'white');
toc

figure
imshow(img1_marked);
figure
imshow(img2_marked);
