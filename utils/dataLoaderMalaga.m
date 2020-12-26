classdef dataLoaderMalaga < handle
    %DATALOADERMALAGA data loader for malaga dataset
    properties
        camParams
        last_frame
        index
        dataset_path
        left_images_dirs
        ground_truth_data
        finished
    end
    
    methods
        function obj = dataLoaderMalaga(path)
            %Parking dataset loader
            obj.dataset_path = path;
            [obj.camParams,...
                obj.ground_truth_data,...
                obj.left_images_dirs,...
                obj.last_frame] = obj.loadGeneralData();
            obj.index = 0;
            obj.finished = false;
        end
        
        % Gets next image and ground truth pose
        function [next_img, gt_pose] = next(obj)
            % Error if already loaded all files
            assert(obj.index <= obj.last_frame);
            next_img = imread(strcat(obj.left_images_dirs(obj.index+1).folder,...
                "/",obj.left_images_dirs(obj.index+1).name));
            gt_pose = [];
            obj.index = obj.index + 1;
            if obj.index > obj.last_frame
                obj.finished = true;
            end
        end
        
        % Retrieves a specific frame
        function [img, ground_truth_pose] = retrieveFrame(obj, index)
            % Error if invalid index
            assert(index <= obj.last_frame);
            img = imread(strcat(obj.left_images_dirs(index+1).folder,...
                "/",obj.left_images_dirs(index+1).name));
            ground_truth_pose = [];
        end
        
        % Load camera parameters gt_position and last_frame index
        function [camParams, ground_truth, left_images_dirs, last_frame] = loadGeneralData(obj)
            K = [621.18428 0 404.0076
                0 621.18428 309.05989
                0 0 1];
            camParams = cameraParameters('IntrinsicMatrix', K.');
            ground_truth = [];
            images = dir(strcat(obj.dataset_path,...
                '/malaga-urban-dataset-extract-07_rectified_800x600_Images'));
            left_images_dirs = images(3:2:end);
            last_frame = length(left_images_dirs) - 1;
        end
        
        % Resets object to the start point
        function [] = reset(obj)
            obj.index = 0;
            obj.finished = false;
        end
    end
end

