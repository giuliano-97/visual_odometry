close all
clear

[cameraParams, ~, ~] = loadGeneralData(2); %parking

img0_file = "./data/parking/images/img_00200.png";
img1_file = "./data/parking/images/img_00202.png";
img2_file = "./data/parking/images/img_00203.png";

img0 = imread(img0_file);
img1 = imread(img1_file);
img2 = imread(img2_file);
img0_gray = rgb2gray(img0);
img1_gray = rgb2gray(img1);
img2_gray = rgb2gray(img2);


klt_tracker = KLTTracker('NumPyramidLevels',4,...
                         'MaxBidirectionalError', 2,...
                         'BlockSize', [51, 51],...
                         'MaxIterations', 1000);


[keypoints, landmarks, cam2Matrix] = bootstrap(img0_gray,...
                               img1_gray,...
                               cameraParams);

state.landmarks = landmarks;
state.keypoints = keypoints;
state.candidate_keypoints = [];
state.candidate_first_keypoints = [];
state.candidate_first_poses = [];
state.candidate_time_indxs = [];
                           
[new_landmarks,...
    new_keypoints,...
    curr_pose] = updateW2D3D(img1_gray,...
                             img2_gray,...
                             state,...
                             cameraParams,...
                             klt_tracker);

                         
T0 = [eye(3);[0,0,0]];
T1 = cam2Matrix;
T2 = curr_pose;

                         
figure(1);
subplot(1,3,1);

% Plot landmarks in 3D
plot3(landmarks(:,1), landmarks(:,2), landmarks(:,3),'*');
hold on;

% Get orientation and position for both views
orientation0 = eye(3); position0 = zeros(1,3);
orientation1 = T1(1:3,:); position1 = T1(end,:);
orientation2 = T2(1:3,:); position2 = T2(end,:);
% % Convert to rigid body transform object
% absPose0 = rigid3d(orientation0, position0);
% absPose1 = rigid3d(orientation1, position1);
% Plot the cameras
plotCamera('Orientation', orientation0, 'Location', position0, 'Size', 1);
text(0,0,0,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
plotCamera('Orientation', orientation1, 'Location', position1, 'Size', 1);
text(position1(1), position1(2), position1(3),'Cam 2','fontsize',10,'color','k','FontWeight','bold');
plotCamera('Orientation', orientation2, 'Location', position2, 'Size', 1);
text(position2(1), position2(2), position2(3),'Cam 3','fontsize',10,'color','k','FontWeight','bold');

%     set(gca,'CameraUpVector',[0 0 1]);
axis equal
grid

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

% Get matched points locations
p1 = state.keypoints';
p2 = new_keypoints';

% Display matched points
subplot(1,3,2)
imshow(img1,[]);
hold on
plot(p1(1,:), p1(2,:), 'ys');
title('Image 1')

subplot(1,3,3)
imshow(img2,[]);
hold on
plot(p2(1,:), p2(2,:), 'ys');
title('Image 2')
