classdef VisualOdometry
    %VISUALODOMETRY Implementation of a simple VO pipeline
    
    properties
        % Constant params
        cameraParams
        angularThreshold
        maxTemporalRecall
        keypointsMode string
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
                optionalArgs.KeypointsMode string ...
                    {mustBeMember(optionalArgs.KeypointsMode,...
                    {'KLT', 'Matched'})} = 'KLT'
            end
            obj.cameraParams = cameraParams;
            obj.angularThreshold = optionalArgs.angularThreshold;
            obj.maxTemporalRecall = optionalArgs.maxTemporalRecall;
            obj.keypointsMode = optionalArgs.KeypointsMode;
            obj.tracker = KLTTracker();
        end
        
        function [curr_state, curr_pose, pose_status] = processFrame(obj, prev_img, ...
                curr_img, prev_state)
            % PROCESSFRAME Summary of this method goes here
            %   TODO: add detailed explanation
            arguments
                obj
                prev_img
                curr_img % The new image
                prev_state
            end
            %% Detect Harris features in the new image
            % Detect keypoints in the new image - will need anyways later
            % for new candidate keypoints selection
            curr_keypoints = detectHarrisFeatures(curr_img);
            
            %% Estimate camera pose from 2D-3D point correspondences
            
            if strcmp(obj.keypointsMode, 'KLT')
                % Instantiate KLT tracker and initialize
                [curr_pts, val_idx, ~] = obj.tracker.track(prev_img,...
                    curr_img, prev_state.keypoints);
                % Estimate the pose in world coordinates
                [R_WC, T_WC, inl_indx, pose_status] = estimateWorldCameraPose(...
                    curr_pts(val_idx,:), prev_state.landmarks(val_idx,:),...
                    obj.cameraParams, ...
                    'MaxNumTrials', 5000, 'Confidence', 95, ...
                    'MaxReprojectionError', 3);
                % Keep only inliers from PnP
                curr_state.landmarks = prev_state.landmarks(val_idx(inl_indx), :);
                curr_state.keypoints = prev_state.keypoints(val_idx(inl_indx), :);
                
            elseif strcmp(obj.keypointsMode, 'Matched')
                % Define cornerPoins struct with the previous keypoints
                prev_keypoints = cornerPoints(prev_state.keypoints);
                % Extract FREAK descriptors - default for corner points
                [prev_descriptors, ~] = extractFeatures(prev_img, ...
                    prev_keypoints);
                [curr_descriptors, curr_keypoints] = extractFeatures(curr_img, ...
                    curr_keypoints);
                % Match Harris features in the new image
                indexPairs = matchFeatures(prev_descriptors, ...
                    curr_descriptors);
                % Get set of keypoints for new image
                curr_pts = curr_keypoints.Location(indexPairs(:,2),:);
                curr_landmarks = prev_state.landmarks(indexPairs(:,1),:);
                % Estimate pose in world coordinates
                [R_WC, T_WC, inl_indx pose_status] = estimateWorldCameraPose(...
                    curr_pts, curr_landmarks, obj.cameraParams, ...
                    'MaxNumTrials', 4000, 'Confidence', 90, ...
                    'MaxReprojectionError', 2.0);
                % Keep only inliers from PnP
                curr_state.landmarks = curr_landmarks(inl_indx, :);
                curr_state.keypoints = curr_pts(inl_indx, :);
            end
            
            % If ignore frame if pose estimation failed
            if pose_status > 0
                curr_pose = [eye(3), zeros(1,3)];
                curr_state = prev_state;
                return
            end
            
            % Update pose
            curr_pose = [R_WC;T_WC];
            
            %% Triangulate new landmarks
            [curr_state, tracked_keypoints] = candidateTriangulation(prev_img,...
                prev_state, curr_img, curr_pose, obj.cameraParams, obj.tracker);
            %% Select new keypoints to track
            % Select subset of new keypoints
            validIndex = selectCandidateKeypoints(...
                tracked_keypoints, curr_keypoints.Location);
            new_candidate_keypoints = curr_keypoints.Location(validIndex, :);
            
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

