classdef dataLoaderKitti < handle
    %DATALOADERKITTI data loader for kitti dataset
    properties
        camParams
        last_frame
        index
        dataset_path
        img_file_format
        ground_truth_data
        finished
    end
    
    methods
        function obj = dataLoaderKitti(path)
            %KITTI dataset loader
            obj.dataset_path = path;
            [obj.camParams,...
                obj.ground_truth_data,...
                obj.last_frame] = obj.loadGeneralData();
            obj.index = 0;
            obj.img_file_format = strcat(obj.dataset_path,'/00/image_0/%06d.png');
            obj.finished = false;
        end
        
        % Gets next image and ground truth pose
        function [next_img, gt_pose] = next(obj)
            % Error if already loaded all files
            assert(obj.index <= obj.last_frame);
            next_img = imread(sprintf(obj.img_file_format,obj.index));
            gt_pose = obj.ground_truth_data(obj.index+1, :);
            obj.index = obj.index + 1;
            if obj.index > obj.last_frame
                obj.finished = true;
            end
        end
        
        % Retrieves a specific frame
        function [img, ground_truth_pose] = retrieveFrame(obj, index)
            % Error if invalid index
            assert(index <= obj.last_frame);
            img = imread(sprintf(obj.img_file_format,index));
            ground_truth_pose = obj.ground_truth_data(obj.index+1, :);            
        end
        
        % Load camera parameters
        function [camParams, ground_truth, last_frame] = loadGeneralData(obj)
            ground_truth = load(strcat(obj.dataset_path, '/poses/00.txt'));
            ground_truth = ground_truth(:, [end-8 end]);
            last_frame = 4540;
            K = [7.188560000000e+02 0 6.071928000000e+02
                0 7.188560000000e+02 1.852157000000e+02
                0 0 1];
            camParams = cameraParameters('IntrinsicMatrix',K.');
        end
        
        % Resets object to the start point
        function [] = reset(obj, index)
            arguments
                obj
                index int32 = 0;
            end
            assert(index <= obj.last_frame, 'Index out of range');
            obj.index = index;
            obj.finished = false;
        end
    end
end

