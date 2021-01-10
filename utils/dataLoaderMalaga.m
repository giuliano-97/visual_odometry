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

        % Parameters tunned for dataset
        bootstrap_MinNumLandmarks =             200
        bootstrap_MaxDepth =                    200
        bootstrap_FeatureMatchingMode =         'KLT'
        bootstrap_FilterSize =                  7
        bootstrap_MinQuality =                  0.005
        
        vo_MaxTemporalRecall =                  15
        vo_MaxNumLandmarks =                    200
        vo_MaxReprojectionError =               2
        vo_AngularThreshold =                   1.0
        vo_PenaltyFactor =                      0.4
        vo_UniformityScoreSigma =               20
        
        vo_KLTnumPyramidLevels =                6
        vo_KLTmaxBidirectionalError double =    3
        vo_KLTblockSize double =                [51 51]
        vo_KLTmaxIterations double =            100
        
        vo_RANSACMaxNumTrials double =          4000
        vo_RANSACConfidence double =            99
        vo_RANSACMAxReprojectionError double =  3
        
        vo_NewCandidateMinQuality double =          0.004
        vo_NewCandidateFilterSize double =          7
        vo_NewCandidateMinDistance double =         18
        vo_NewCandidateMaxNewKeypoints double =    100
        
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

