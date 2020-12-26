
dataset_type = 0; % 0: KITTI, 1: malaga, 2: parking, 3:KITTI_tutorial

% Pick the correspoinding data loader
if dataset_type ==0
    data_loader = dataLoaderKitti('./data/kitti');
end

% Iterate over all frames 
figure
while ~data_loader.finished
   [img, gt_pose] = data_loader.next();
   imshow(img);
   display(strcat("GT position: ","[",num2str(gt_pose),"]"))
end