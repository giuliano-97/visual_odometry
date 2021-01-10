
dataset_type = 0; % 0: KITTI, 1: malaga, 2: parking, 3:KITTI_tutorial

% Pick the correspoinding data loader
if dataset_type ==0
    data_loader = dataLoaderKitti('./data/kitti');
elseif dataset_type == 1
    data_loader = dataLoaderMalaga('./data/malaga-urban-dataset-extract-07');
elseif dataset_type == 2
    data_loader = dataLoaderParking('./data/parking');
else
    assert(false, "Invalid dataset type choose: 0, 1,2,3");
end

% Iterate over all frames 
figure
while ~data_loader.finished
   [img, gt_pose] = data_loader.next();
   imshow(img);
   display(strcat(sprintf("GT position %08d: ", data_loader.index-1),"[",num2str(gt_pose),"]"))
end
