classdef VisualOdometry
    %VISUALODOMETRY Implementation of a simple VO pipeline
    
    properties
        % Constant params
        cameraParams
        angularThreshold
        maxTemporalRecall
        maxNumLandmarks
        maxReprojectionError
        tracker
    end
     
    methods        
        function obj = VisualOdometry(cameraParams, optionalArgs)
            %VISUALODOMETRY Constructor
            %   Initialize the visual odometry pipeline
            arguments
                cameraParams
                optionalArgs.angularThreshold double = 1.0
                optionalArgs.maxTemporalRecall uint32 = 20
                optionalArgs.maxNumLandmarks uint32 = 700
                optionalArgs.maxReprojectionError double = 5
            end
            obj.cameraParams = cameraParams;
            obj.angularThreshold = optionalArgs.angularThreshold;
            obj.maxTemporalRecall = optionalArgs.maxTemporalRecall;
            obj.maxNumLandmarks = optionalArgs.maxNumLandmarks;
            obj.tracker = KLTTracker();
        end
        
        function [curr_state, tracked_keypoints] = candidateTriangulation(...
                obj, prev_img, prev_state, curr_img, curr_state, curr_pose)
            arguments
                obj
                prev_img
                prev_state
                curr_img
                curr_state
                curr_pose
            end
            
            %% Tracking current keypoints and landmarks
%             [keypoints_tracked, val_key, ~] = obj.tracker.track(prev_img,...
%                 curr_img,...
%                 prev_state.keypoints);
%             landmarks = prev_state.landmarks(val_key, :);
%             keypoints = keypoints_tracked(val_key, :);
            
            landmarks = curr_state.landmarks;
            keypoints = curr_state.keypoints;
            
            %% Introducing candidates if appropriate
            % Fast-forward candidates by tracking from previous frame
            [candidate_tracked, val_cand, ~] = obj.tracker.track(prev_img,...
                curr_img,...
                prev_state.candidate_keypoints);
            tracked_keypoints = candidate_tracked(val_cand,:);
            
            candidate_keypoints = double.empty(0,2);
            candidate_first_keypoints = double.empty(0,2);
            candidate_first_poses = [{}];
            candidate_time_indxs = [];
            
            % Build camera matrix for the current pose
%             [rotMat1, transVec1] = cameraPoseToExtrinsics(...
%                     curr_pose(1:3,:),...
%                     curr_pose(end,:));
%             cam_mat1 = cameraMatrix(obj.cameraParams, rotMat1, transVec1);            
%             
            % Iterate over all candidates
            for i=find(val_cand.')
                % Triangulate candidate
%                 [rotMat0, transVec0] = cameraPoseToExtrinsics(...
%                     prev_state.candidate_first_poses{i}(1:3,:),...
%                     prev_state.candidate_first_poses{i}(end,:));
%                 cam_mat0 = cameraMatrix(obj.cameraParams, rotMat0, transVec0);
%                 [cand_landmark, repro_err, is_valid] = triangulate(...,
%                     prev_state.candidate_first_keypoints(i,:),...
%                     candidate_tracked(i,:),...
%                     cam_mat0,...
%                     cam_mat1);
                
                %% Triangulate
                % Add views with keypoints to viewset
                vSet = imageviewset;
                vSet = addView(vSet, 1, rigid3d(...
                    [prev_state.candidate_first_poses{i},[0;0;0;1]]),...
                    'Points', prev_state.candidate_keypoints(i,:));
                vSet = addView(vSet, 2, rigid3d(curr_pose(1:3,:),curr_pose(end,:)),...
                    'Points',...
                    candidate_tracked(i,:));
                % Add correspondences to viewset
                vSet = addConnection(vSet, 1, 2, 'Matches', ...
                    [1; 1]');                
                % Find tracks
                tracks = findTracks(vSet);
                
                % Get camera poses
                cameraPoses = poses(vSet);

                % Triangulate points
                K = obj.cameraParams.IntrinsicMatrix;
                focalLength = [K(1,1), K(2,2)];
                principalPoint = [K(3,1), K(3,2)];
                imageSize = size(prev_img);
                intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

                
                [cand_landmark, repro_err, is_valid] = triangulateMultiview(tracks, ...
                    cameraPoses, intrinsics);
                
                
                
                % Keep only valid landmarks i.e. discard the ones which have
                % negative depth, are too far away, or whose reprojection error
                % is higher than the required threshold
                is_valid = is_valid & cand_landmark(:,3) > 0 & ...
                    repro_err <= obj.maxReprojectionError;
               
                
                % Ignore if point behind camera or invalid
                if ~is_valid
                    continue
                end
                
                % Add landmarks if complies baseline threshold
                if calculateAngleDeg(cand_landmark, prev_state.candidate_first_poses{i},...
                        curr_pose) > obj.angularThreshold
                    if size(landmarks, 1) >= obj.maxNumLandmarks
                        landmarks = landmarks(2:end,:);
                        keypoints = keypoints(2:end,:);
                    end
                    landmarks = [landmarks; cand_landmark]; %#ok<*AGROW>
                    keypoints = [keypoints; candidate_tracked(i,:)];
                else
                    % Discard candidate if has been stored for too long
                    if prev_state.candidate_time_indxs(i) > -obj.maxTemporalRecall
                        candidate_keypoints = [candidate_keypoints;...
                            candidate_tracked(i,:)];
                        candidate_first_keypoints = [candidate_first_keypoints;...
                            prev_state.candidate_first_keypoints(i,:)];
                        candidate_first_poses = [candidate_first_poses,...
                            prev_state.candidate_first_poses{i}];
                        candidate_time_indxs = [candidate_time_indxs, ...
                            prev_state.candidate_time_indxs(i)-1];
                    end
                end
                
            end
            
            curr_state.landmarks = landmarks;
            curr_state.keypoints = keypoints;
            curr_state.candidate_keypoints = candidate_keypoints;
            curr_state.candidate_first_keypoints = candidate_first_keypoints;
            curr_state.candidate_first_poses = candidate_first_poses;
            curr_state.candidate_time_indxs = candidate_time_indxs;
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
            
            %% Estimate camera pose from 2D-3D point correspondences
            
            % Instantiate KLT tracker and initialize
            [tracked_keypoints, val_idx, ~] = obj.tracker.track(prev_img,...
                curr_img, prev_state.keypoints);
            valid_tracked_keypoints = tracked_keypoints(val_idx,:);
            valid_landmarks = prev_state.landmarks(val_idx,:);
            
            % Estimate the pose in world coordinates
            [R_WC, T_WC, inl_idx, pose_status] = estimateWorldCameraPose(...
                double(valid_tracked_keypoints), double(valid_landmarks),...
                obj.cameraParams, ...
                'MaxNumTrials', 5000, 'Confidence', 98, ...
                'MaxReprojectionError', 2);
            
            % Keep only RANSAC inliers
            curr_state.keypoints = valid_tracked_keypoints(inl_idx,:);
            curr_state.landmarks = valid_landmarks(inl_idx,:);
                        
            % If ignore frame if pose estimation failed
%             if pose_status > 0
%                 curr_pose = [eye(3), zeros(1,3)];
%                 curr_state = prev_state;
%                 return
%             end
            
            % Update pose
            curr_pose = [R_WC;T_WC];
            
            %% Triangulate new landmarks
            tic
            [curr_state, tracked_keypoints] = obj.candidateTriangulation(...
                prev_img, prev_state, curr_img, curr_state, curr_pose);
            toc
            
            %% Select new keypoints to track
            % Only select new keypoints if the number of landmarks which
            % are being tracked is smaller than the allowed maximum
            new_candidate_keypoints = selectCandidateKeypoints(curr_img,...
                [curr_state.keypoints; curr_state.candidate_keypoints],...
                'MaxNewKeypoints', 300,...
                'MinQuality', 0.001, ...
                'FilterSize', 5, ...
                'MinDistance', 10,...
                'FractionToKeep', 0.8);

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

