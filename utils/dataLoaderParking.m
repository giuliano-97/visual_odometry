classdef dataLoaderParking < handle
    %DATALOADERPARKING data loader for parking dataset
    properties
        camParams
        last_frame
        index
        dataset_path
        img_file_format
        ground_truth_data
        finished
        
        % Parameters tunned for dataset
        bootstrap_frames =                      [1,3]
        bootstrap_MaxNumKeypoints =             2000
        bootstrap_MinNumLandmarks =             200
        bootstrap_MaxDepth =                    200
        bootstrap_FeatureMatchingMode =         'KLT'
        bootstrap_FilterSize =                  7
        bootstrap_MinQuality =                  0.001
        
        vo_MaxTemporalRecall =                  15
        vo_MaxNumLandmarks =                    200
        vo_MaxReprojectionError =               4
        vo_AngularThreshold =                   1.5
        vo_PenaltyFactor =                      0.5
        vo_UniformityScoreSigma =               30
        
        vo_KLTnumPyramidLevels =                7
        vo_KLTmaxBidirectionalError double =    2
        vo_KLTblockSize double =                [51 51]
        vo_KLTmaxIterations double =            100
        
        vo_RANSACMaxNumTrials double =          1000
        vo_RANSACConfidence double =            99
        vo_RANSACMAxReprojectionError double =  3
        
        vo_NewCandidateMinQuality double =          0.001
        vo_NewCandidateFilterSize double =          5
        vo_NewCandidateMinDistance double =         18
        vo_NewCandidateMaxNewKeypoints double =    50
        
    end
    
    methods
        function obj = dataLoaderParking(path)
            %Parking dataset loader
            obj.dataset_path = path;
            [obj.camParams,...
                obj.ground_truth_data,...
                obj.last_frame] = obj.loadGeneralData();
            obj.index = 0;
            obj.img_file_format = strcat(obj.dataset_path,'/images/img_%05d.png');
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
        
        % Load camera parameters gt_position and last_frame index
        function [camParams, ground_truth, last_frame] = loadGeneralData(obj)
            K = load(strcat(obj.dataset_path, '/K.txt'));
            camParams = cameraParameters('IntrinsicMatrix', K.');
            ground_truth = load(strcat(obj.dataset_path, '/poses.txt'));
            ground_truth = ground_truth(:, [end-8 end]);
            last_frame = 598;
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

