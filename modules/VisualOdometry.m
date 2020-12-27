classdef VisualOdometry
    %VISUALODOMETRY Implementation of a simple VO pipeline
    
    properties
        % Constant params
        cameraParams
        angularThreshold
        maxTemporalRecall
        maxNumLandmarks
        tracker
    end
    
    methods
        function obj = VisualOdometry(cameraParams, optionalArgs)
            %VISUALODOMETRY Constructor
            %   Initialize the visual odometry pipeline
            arguments
                cameraParams
                optionalArgs.angularThreshold double = 5
                optionalArgs.maxTemporalRecall uint32 = 20
                optionalArgs.maxNumLandmarks uint32 = 500
            end
            obj.cameraParams = cameraParams;
            obj.angularThreshold = optionalArgs.angularThreshold;
            obj.maxTemporalRecall = optionalArgs.maxTemporalRecall;
            obj.maxNumLandmarks = optionalArgs.maxNumLandmarks;
            obj.tracker = KLTTracker();
        end
        
        function [curr_state, curr_pose] = processFrame(obj, prev_img, ...
                curr_img, prev_state)
            % PROCESSFRAME Summary of this method goes here
            %   TODO: add detailed explanation
            arguments
                obj
                prev_img
                curr_img % The new image
                prev_state
            end
            
            %% Estimate camera pose from 2D-3D point correspondences
            
            % Instantiate KLT tracker and initialize
            [curr_pts, val_idx, ~] = obj.tracker.track(prev_img,...
                curr_img, prev_state.keypoints);
            % Estimate the pose in world coordinates
            [R_WC, T_WC, inl_indx] = estimateWorldCameraPose(...
                curr_pts(val_idx,:), prev_state.landmarks(val_idx,:),...
                obj.cameraParams, ...
                'MaxNumTrials', 5000, 'Confidence', 95, ...
                'MaxReprojectionError', 3);
            % Keep only inliers from PnP
            curr_state.landmarks = prev_state.landmarks(val_idx(inl_indx), :);
            curr_state.keypoints = prev_state.keypoints(val_idx(inl_indx), :); %#ok<STRNU>
            
            % Update pose
            curr_pose = [R_WC;T_WC];
            
            %% Triangulate new landmarks
            [curr_state, tracked_keypoints] = candidateTriangulation(prev_img,...
                prev_state, curr_img, curr_pose, obj.cameraParams, obj.tracker);
            
            %% Select new keypoints to track
            % Only select new keypoints if the number of landmarks which
            % are being tracked is smaller than the allowed maximum
            if length(curr_state.keypoints) < obj.maxNumLandmarks
                new_candidate_keypoints = selectCandidateKeypoints(curr_img,...
                    [curr_state.keypoints; curr_state.candidate_keypoints],...
                    'MaxNewKeypoints', 50, 'MinQuality', 0.01, ...
                    'MinDistance', 30);

                % Appending candidates to keypoints to track
                curr_state.candidate_keypoints = [curr_state.candidate_keypoints;...
                    new_candidate_keypoints];
                curr_state.candidate_first_keypoints = [curr_state.candidate_first_keypoints;...
                    new_candidate_keypoints];
                curr_state.candidate_first_poses = [curr_state.candidate_first_poses,...
                    repmat({curr_pose},1,size(new_candidate_keypoints,1))];
                curr_state.candidate_time_indxs = [curr_state.candidate_time_indxs,...
                    zeros(1, size(new_candidate_keypoints,1))];
            end
        end
    end
end

