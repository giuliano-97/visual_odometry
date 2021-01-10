% Compare Harris corner detector with Shi-Tomasi

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

% Load the first bootstrap frame
bootstrap_frames = data_loader.bootstrap_frames;
img1 = data_loader.retrieveFrame(bootstrap_frames(1));
if ndims(img1) == 3
   img1 = rgb2gray(img1);
end

% Extract the keypoints
points_gftt = detectMinEigenFeatures(img1,'MinQuality', 0.001, 'FilterSize', 5);
points_gftt = selectUniform(points_gftt, 100, size(img1));
points_harris = detectHarrisFeatures(img1,'MinQuality', 0.001, 'FilterSize', 5);
points_harris = selectUniform(points_harris, 100, size(img1));

% Show the results
figure(1);
subplot(1,2,1);
imshow(insertMarker(img1, points_gftt.Location));
title("Shi-Tomasi");
subplot(1,2,2);
imshow(insertMarker(img1, points_harris.Location));
title("Harris");