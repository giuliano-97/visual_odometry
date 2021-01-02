classdef VisualOdometry
    %VISUALODOMETRY Implementation of a simple VO pipeline
    
    properties
        % Constant params
        cameraParams
        imageSize
        angularThreshold
        maxTemporalRecall
        maxNumLandmarks
        maxReprojectionError
        tracker
    end
     
    methods        
        function obj = VisualOdometry(cameraParams, imageSize, optionalArgs)
            %VISUALODOMETRY Constructor
            %   Initialize the visual odometry pipeline
            arguments
                cameraParams
                imageSize
                optionalArgs.angularThreshold double = 1.0
                optionalArgs.maxTemporalRecall int8 = 10
                optionalArgs.maxNumLandmarks uint32 = 300
                optionalArgs.maxReprojectionError double = 2
            end
            obj.cameraParams = cameraParams;
            obj.imageSize = imageSize;
            obj.angularThreshold = optionalArgs.angularThreshold;
            obj.maxTemporalRecall = optionalArgs.maxTemporalRecall;
            obj.maxNumLandmarks = optionalArgs.maxNumLandmarks;
            obj.maxReprojectionError = optionalArgs.maxReprojectionError;
            obj.tracker = KLTTracker(...
                'NumPyramidLevels', 4,...
                'MaxBidirectionalError', 3,...
                'BlockSize', [51 51],...
                'MaxIterations', 50);
        end
        
        function intrinsics = getCameraIntrinsics(obj)
                    % Build camera intrinsics
            K = obj.cameraParams.IntrinsicMatrix;
            focalLength = [K(1,1), K(2,2)];
            principalPoint = [K(3,1), K(3,2)];
            intrinsics = cameraIntrinsics(focalLength, principalPoint, obj.imageSize);
        end
        
        function err = computeReprojectionError(obj, landmarks, keypoints, cameraPose)
            % TODO: add sanity check on the dimensions of input arrays
            
            % Compute camera extrinsics and intrinsics
            [rotMat, transVec] = cameraPoseToExtrinsics(cameraPose(1:3,1:3),...
                cameraPose(4,:));
            intrinsics = obj.getCameraIntrinsics();
            % Reproject landmarks
            keypoints_reprojected = worldToImage(intrinsics, rotMat, transVec,...
                landmarks);
            % Compute reprojection error
            diff = keypoints - keypoints_reprojected;
            err = vecnorm(diff', 2)';
        end
        
        function [curr_state, tracked_keypoints] = candidateTriangulation(...
                obj, prev_img, prev_state, curr_img, curr_state, curr_pose)
                        
            landmarks = curr_state.landmarks;
            keypoints = curr_state.keypoints;
            reproError = curr_state.reproError;
            reproErrorOld = reproError;

            assert(length(reproError)==size(landmarks,1), 'Inconsistent state');
%             assert(size(landmarks,1)>0, 'No landmarks');
%             assert(length(reproError)>0, 'No reprojecton Error');
            
            fprintf('\tPrevious candidates: \t%d\n', size(prev_state.candidate_keypoints,1));

            % Track candidates from the previous frame
            [candidate_tracked, val_cand, ~] = obj.tracker.track(prev_img,...
                curr_img, prev_state.candidate_keypoints);
            tracked_keypoints = candidate_tracked(val_cand,:);

            fprintf('\tTracked candidates: \t%d\n', size(tracked_keypoints,1));
            
            candidate_keypoints = double.empty(0,2);
            candidate_first_keypoints = double.empty(0,2);
            candidate_first_poses = [{}];
            candidate_time_indxs = [];
            candidate_min_score = 0;

            % Build the camera matrix for the current view
            [rotMat1, transVec1] = cameraPoseToExtrinsics(...
                    curr_pose(1:3,:),...
                    curr_pose(end,:));
            cam_mat1 = cameraMatrix(obj.cameraParams, rotMat1, transVec1);
            
            % Iterate over all candidates
            for i=find(val_cand.')
                % Updating reprojection errors of landmarks
                if size(reproError,1)>0
                    [max_reproError, max_reproError_indx] = max(reproErrorOld);
                    max_reproError_indx = max_reproError_indx(1);
                else
                    max_reproError = Inf;
                end
                    
                % Triangulate candidate
                [rotMat0, transVec0] = cameraPoseToExtrinsics(...
                    prev_state.candidate_first_poses{i}(1:3,:),...
                    prev_state.candidate_first_poses{i}(end,:));
                cam_mat0 = cameraMatrix(obj.cameraParams, rotMat0, transVec0);
                [cand_landmark, repro_err, is_valid] = triangulate(...,
                    prev_state.candidate_first_keypoints(i,:),...
                    candidate_tracked(i,:),...
                    cam_mat0,...
                    cam_mat1);

                % Keep only valid landmarks i.e. discard the ones which have
                % negative depth, are too far away, or whose reprojection error
                % is higher than the required threshold
                is_valid = is_valid & repro_err <= obj.maxReprojectionError;
                
                

                % Ignore if point behind camera
                if ~is_valid
                    continue
                end
                % Add landmarks if complies baseline threshold
                if calculateAngleDeg(cand_landmark, prev_state.candidate_first_poses{i},...
                        curr_pose) > obj.angularThreshold
%                         && size(landmarks, 1) < obj.maxNumLandmarks
                    if size(landmarks, 1) >= obj.maxNumLandmarks...
                            && repro_err <= max_reproError
                        
                        landmarks(max_reproError_indx,:) = cand_landmark;
                        keypoints(max_reproError_indx,:) = candidate_tracked(i,:);
                        reproError(max_reproError_indx,:) = repro_err;
                        reproErrorOld(max_reproError_indx,:) = 0;
                    end
                    if size(landmarks, 1) < obj.maxNumLandmarks
                        landmarks = [landmarks; cand_landmark]; %#ok<*AGROW>
                        keypoints = [keypoints; candidate_tracked(i,:)];
                        reproError = [reproError; repro_err];
                    end
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
            curr_state.reproError = reproError;
            curr_state.candidate_keypoints = candidate_keypoints;
            curr_state.candidate_first_keypoints = candidate_first_keypoints;
            curr_state.candidate_first_poses = candidate_first_poses;
            curr_state.candidate_time_indxs = candidate_time_indxs;            
        end
            
        function [curr_state, tracked_keypoints] = candidateTriangulationV2(...
                obj, prev_img, prev_state, curr_img, curr_state, curr_pose)
            arguments
                obj
                prev_img
                prev_state
                curr_img
                curr_state
                curr_pose
            end

            % Fetch current landmarks and keypoints
            landmarks = curr_state.landmarks;
            keypoints = curr_state.keypoints;
            
            % Fast-forward candidates by tracking from previous frame
            [candidates_tracked, val_cand, ~] = obj.tracker.track(prev_img,...
                curr_img,...
                prev_state.candidate_keypoints);
            tracked_keypoints = candidates_tracked(val_cand,:);
            
            % Build camera intrinsics
            intrinsics = obj.getCameraIntrinsics();
            
            % Create new empty viewset
            vSet = imageviewset;
            
            % Add the last view
            last_view_id = uint32(obj.maxTemporalRecall + 1);
            vSet = addView(vSet, last_view_id, ...
                rigid3d(curr_pose(1:3, 1:3), curr_pose(4,:)), ...
                'Points', candidates_tracked);
            
            % Initialize a cell array to store the assignment of keypoints
            % to each of the previous N views
            bins = cell(obj.maxTemporalRecall, 2);
            % Iterate over and bin all the valid keypoints
            for i=find(val_cand.')
                % Compute the view id of this keypoint
                view_id = prev_state.candidate_time_indxs(i) + last_view_id;
                % If there are no points already assigned add pose
                if isempty(bins{view_id,1})
                    bins{view_id,2} = prev_state.candidate_first_poses{i};
                end
                % Bin the keypoint
                bins{view_id,1} = [bins{view_id,1}, i];
            end
            
            candidate_keypoints = double.empty(0,2);
            candidate_first_keypoints = double.empty(0,2);
            candidate_first_poses = [{}];
            candidate_time_indxs = [];
            
            % Loop over non-empty bins and add views and connections
            for i=1:obj.maxTemporalRecall
                if ~isempty(bins{i,1})
                    % Get all the candidate keypoints
                    kps_idxs = bins{i,1};
                    kps = prev_state.candidate_first_keypoints(kps_idxs,:);
                    % Add view
                    pose = bins{i,2};
                    vSet = addView(vSet, i, ...
                        rigid3d(pose(1:3, 1:3), pose(4,:)), ...
                        'Points', kps);
                    % Add connection to the last view
                    idx_pairs = [1:numel(kps_idxs); kps_idxs]';
                    vSet = addConnection(vSet, i, last_view_id, ...
                        'Matches', idx_pairs);
                    % Find tracks and poses for this and the last view
                    view_ids = [i, last_view_id];
                    tracks = findTracks(vSet, view_ids);
                    cameraPoses = poses(vSet, view_ids);
                    % Triangulate
                    [cand_landmarks, repro_errs, is_valid] = triangulateMultiview(tracks, ...
                        cameraPoses, intrinsics);
                    is_valid = is_valid & ...
                        repro_errs < obj.maxReprojectionError & ...
                        cand_landmarks(:,3) > 0;
                    % Validate the landmarks
                    for j=find(is_valid.')
                        idx = kps_idxs(j);
                        cand_landmark = cand_landmarks(j,:);
                        angle = calculateAngleDeg(cand_landmark, pose, curr_pose);
                        if angle > obj.angularThreshold
                            if size(landmarks, 1) < obj.maxNumLandmarks
                                landmarks = [landmarks; cand_landmark]; %#ok<*AGROW>
                                keypoints = [keypoints; candidates_tracked(idx,:)];
                            end
                        % Discard candidate if it has been stored for too long
                        elseif prev_state.candidate_time_indxs(idx) > -obj.maxTemporalRecall
                            candidate_keypoints = [candidate_keypoints;...
                                candidates_tracked(idx,:)];
                            candidate_first_keypoints = [candidate_first_keypoints;...
                                prev_state.candidate_first_keypoints(idx,:)];
                            candidate_first_poses = [candidate_first_poses,...
                                prev_state.candidate_first_poses{idx}];
                            candidate_time_indxs = [candidate_time_indxs, ...
                                prev_state.candidate_time_indxs(idx)-1];
                        end
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
            fprintf('Iteration\n');
            % Instantiate KLT tracker and initialize
            [tracked_keypoints, val_idx, ~] = obj.tracker.track(prev_img,...
                curr_img, prev_state.keypoints);
            valid_tracked_keypoints = tracked_keypoints(val_idx,:);
            valid_landmarks = prev_state.landmarks(val_idx, :);
            fprintf('\tPrevious landmarks: \t%d\n',size(tracked_keypoints,1));
            fprintf('\tTracked landmarks: \t%d\n',size(valid_landmarks,1));
            
            % Estimate the pose in world coordinates
            [R_WC, T_WC, inl_idx, pose_status] = estimateWorldCameraPose(...
                double(valid_tracked_keypoints), double(valid_landmarks),...
                obj.cameraParams, ...
                'MaxNumTrials', 5000, 'Confidence', 99, ...
                'MaxReprojectionError', 2);
            
            % Pose non-linear refinement
            intrinsics = obj.getCameraIntrinsics();
            ViewId = uint32(1); AbsolutePose = rigid3d(R_WC, T_WC);
            pointTracks = repmat(pointTrack(1, [0,0]), length(valid_landmarks),1);
            for i=1:length(valid_landmarks)
                pointTracks(i).ViewIds = ViewId;
                pointTracks(i).Points = valid_tracked_keypoints(i,:);
            end
            cameraPoses = table(ViewId, AbsolutePose);
            [valid_landmarks, refinedPoses, reproErr] =  ...
                bundleAdjustment(valid_landmarks, pointTracks,...
                cameraPoses, intrinsics);
            
            R_WC = refinedPoses.AbsolutePose.Rotation;
            T_WC = refinedPoses.AbsolutePose.Translation;

%             % Verify which landmarks are still in front of the camera (ifc)
%             [orientation, location] = cameraPoseToExtrinsics(R_WC, T_WC);
%             camMat = cameraMatrix(obj.cameraParams, orientation, location);
%             ifc_idx = isInFrontOfCamera(camMat, valid_landmarks);
            
%             reproErr = obj.computeReprojectionError(...
%                 valid_landmarks, valid_tracked_keypoints, ...
%                 [R_WC; T_WC]);
            
            val_idx = reproErr < obj.maxReprojectionError;

            % Update keypoints and landmarks (keep only inliers)
%             curr_state.keypoints = valid_tracked_keypoints(inl_idx,:);
%             curr_state.landmarks = valid_landmarks(inl_idx,:);
            
            curr_state.keypoints = valid_tracked_keypoints(val_idx,:);
            curr_state.landmarks = valid_landmarks(val_idx,:);
            
            fprintf('\tFiltered with err: \t%d\n',size(curr_state.landmarks,1));
            
            % Update the reprojection error of the tracked landmarks
%             curr_state.reproError = obj.computeReprojectionError(...
%                 curr_state.landmarks, curr_state.keypoints, ...
%                 [R_WC; T_WC]);
            curr_state.reproError = reproErr(val_idx);
            
            % Update pose
            curr_pose = [R_WC;T_WC];
            
            %% Triangulate new landmarks
            [curr_state, tracked_keypoints] = obj.candidateTriangulation(...
                prev_img, prev_state, curr_img, curr_state, curr_pose);
            
            %% Select new keypoints to track
            % Only select new keypoints if the number of landmarks which
            % are being tracked is smaller than the allowed maximum
            new_candidate_keypoints = selectCandidateKeypoints(curr_img,...
                [curr_state.keypoints; curr_state.candidate_keypoints],...
                'MinQuality', 0.001, ...
                'FilterSize', 5, ...
                'MinDistance',10,...
                'CandidatesToKeep', 100);

            fprintf('\t Curr state fast forwarded candidates: %d\n',size(curr_state.candidate_keypoints,1));
            
            % Appending candidates to keypoints to track
            curr_state.candidate_keypoints = [curr_state.candidate_keypoints;...
                new_candidate_keypoints];
            curr_state.candidate_first_keypoints = [curr_state.candidate_first_keypoints;...
                new_candidate_keypoints];
            curr_state.candidate_first_poses = [curr_state.candidate_first_poses,...
                repmat({curr_pose},1,size(new_candidate_keypoints,1))];
            curr_state.candidate_time_indxs = [curr_state.candidate_time_indxs,...
                -1*ones(1, size(new_candidate_keypoints,1))];
            
            fprintf('\tAdded new landmarks: \t%d\n\n',size(curr_state.landmarks,1));
            
        end
    end
end

