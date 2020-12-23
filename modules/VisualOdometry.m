classdef VisualOdometry
    %VISUALODOMETRY Implementation of a simple VO pipeline
    
    properties
        % Constant params
        cameraParams
        angularThreshold
        maxTemporalRecall
    end
    
    methods
        function obj = VisualOdometry(cameraParams, optionalArgs)
            %VISUALODOMETRY Constructor
            %   Initialize the visual odometry pipeline
            arguments
                cameraParams
                optionalArgs.angularThreshold double = 5
                optionalArgs.maxTemporalRecall uint32 = 20
            end
            obj.cameraParams = cameraParams;
            obj.angularThreshold = optionalArgs.angularThreshold;
            obj.maxTemporalRecall = optionalArgs.maxTemporalRecall;
        end
        
        function [curr_state, pose] = processFrame(obj, prev_img, ...
                curr_img, prev_state)
            % PROCESSFRAME Summary of this method goes here
            %   TODO: add detailed explanation
            arguments
                obj
                prev_img
                curr_img % The new image
                prev_state
            end
            
            %% Estimate camera pose from tracked points correspondences
            
            % Instantiate KLT tracker and initialize
            tracker = vision.PointTracker();
            initialize(tracker, prev_state.keypoints, prev_img);
            [curr_pts, val_idx, ~] = tracker(curr_img);

            % Estimate the pose in world coordinates
            [R_WC, T_WC, inl_indx] = estimateWorldCameraPose(...
                curr_pts(val_idx,:), prev_state.landmarks(val_idx,:),...
                obj.cameraParams, ...
                'MaxNumTrials', 5000, 'Confidence', 95, ...
                'MaxReprojectionError', 3);

            % Keep only inliers from PnP
            curr_state.landmarks = prev_state.landmarks(val_idx(inl_indx), :);
            curr_state.keypoints = prev_state.keypoints(val_idx(inl_indx), :);
            
            % Update pose
            pose = [R_WC;T_WC];
            
            %% Triangulate new landmarks
            
            %% Select new keypoints to track
            % Detect keypoints in the new image
            newKeypoints = detectHarrisFeatures(curr_img);
            
            % Select subset of new keypoints
            validIndex = selectCandidateKeypoints(...
                curr_state.keypoints, newKeypoints.Location);
            candidateKeypoints = newKeypoints.Location(validIndex, :);
            
            % TODO: append candidates to keypoints to track
        end
    end
end

