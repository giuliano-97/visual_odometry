function plotCameraPose(cameraPose, cameraName)
%PLOTCAMERAPOSE Summary of this function goes here
%   Detailed explanation goes here

% Convert to rigid body transform object
orientation = cameraPose(1:3,1:3);
position = cameraPose(4,:);
absPose = rigid3d(orientation, position);
% Plot the camera
plotCamera('AbsolutePose', absPose, 'Size', 1);
text(position(1), position(2), position(3), cameraName, ...
    'fontsize',10,'color','k','FontWeight','bold');
end

