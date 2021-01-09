classdef VisualOdometry
    %VISUALODOMETRY Implementation of a simple VO pipeline
    
    properties
        % General camera parameters of rectified image
        cameraParams
        imageSize

        % New landmarks params
        angularThreshold
        maxTemporalRecall
        maxNumLandmarks
        maxReprojectionError
        
        % Pose estimation RANSAC params
        ransacMaxNumTrials
        ransacConfidence
        ransacMAxReprojectionError
        
        % New keypoints params
        minNewKeypointsDistance
        maxNewKeypointsPerFrame
        newCandidateMinQuality
        newCandidateFilterSize
        newCandidateMinDistance
        newCandidateMaxNewKeypoints

        % Landmark replacement
        penaltyFactor
        uniformityScoreSigma
        
        % KLT tracker
        tracker

    end
     
    methods        
        function obj = VisualOdometry(cameraParams, imageSize, optionalArgs)
            %VISUALODOMETRY Constructor
            %   Initialize the visual odometry pipeline
            arguments
                cameraParams
                imageSize
                
                optionalArgs.maxTemporalRecall
                optionalArgs.maxNumLandmarks
                optionalArgs.maxReprojectionError
                optionalArgs.angularThreshold
                
                optionalArgs.penaltyFactor
                optionalArgs.uniformityScoreSigma
                
                optionalArgs.KLTnumPyramidLevels
                optionalArgs.KLTmaxBidirectionalError
                optionalArgs.KLTblockSize
                optionalArgs.KLTmaxIterations
                
                optionalArgs.RANSACMaxNumTrials
                optionalArgs.RANSACConfidence
                optionalArgs.RANSACMaxReprojectionError
                
                optionalArgs.NewCandidateMinQuality
                optionalArgs.NewCandidateFilterSize
                optionalArgs.NewCandidateMinDistance
                optionalArgs.NewCandidateMaxNewKeypoints
                
            end
            obj.cameraParams = cameraParams;
            obj.imageSize = imageSize;
            obj.angularThreshold = optionalArgs.angularThreshold;
            obj.maxTemporalRecall = optionalArgs.maxTemporalRecall;
            obj.maxNumLandmarks = optionalArgs.maxNumLandmarks;
            obj.maxReprojectionError = optionalArgs.maxReprojectionError;

            obj.penaltyFactor = optionalArgs.penaltyFactor;
            obj.uniformityScoreSigma = optionalArgs.uniformityScoreSigma;
            
            obj.tracker = KLTTracker(...
                'NumPyramidLevels', optionalArgs.KLTnumPyramidLevels,...
                'MaxBidirectionalError', optionalArgs.KLTmaxBidirectionalError,...
                'BlockSize', optionalArgs.KLTblockSize,...
                'MaxIterations', optionalArgs.KLTmaxIterations);
            
            obj.ransacMaxNumTrials = optionalArgs.RANSACMaxNumTrials;
            obj.ransacConfidence = optionalArgs.RANSACConfidence;
            obj.ransacMAxReprojectionError = optionalArgs.RANSACMaxReprojectionError;
                
            obj.newCandidateMinQuality = optionalArgs.NewCandidateMinQuality;
            obj.newCandidateFilterSize = optionalArgs.NewCandidateFilterSize;
            obj.newCandidateMinDistance = optionalArgs.NewCandidateMinDistance;
            obj.newCandidateMaxNewKeypoints = optionalArgs.NewCandidateMaxNewKeypoints;
            
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
                obj, prev_img, prev_state, curr_img, curr_state, curr_pose,...
                curr_tracked_scores)
                        
            landmarks = curr_state.landmarks;
            keypoints = curr_state.keypoints;
            reproError = curr_state.reproError;
            tracking_scores = curr_tracked_scores;

%             assert(length(reproError)==size(landmarks,1), 'Inconsistent state');
%             assert(size(landmarks,1)>0, 'No landmarks');
%             assert(length(reproError)>0, 'No reprojecton Error');
            
            fprintf('\tPrevious candidates: \t%d\n', size(prev_state.candidate_keypoints,1));

            % Track candidates from the previous frame
            [candidate_tracked, val_cand, candidate_scores] = obj.tracker.track(prev_img,...
                curr_img, prev_state.candidate_keypoints);
            tracked_keypoints = candidate_tracked(val_cand,:);

            fprintf('\tTracked candidates: \t%d\n', size(tracked_keypoints,1));
            
            candidate_keypoints = double.empty(0,2);
            candidate_first_keypoints = double.empty(0,2);
            candidate_first_poses = [{}];
            candidate_time_indxs = [];

            % Build the camera matrix for the current view
            [rotMat1, transVec1] = cameraPoseToExtrinsics(...
                    curr_pose(1:3,:),...
                    curr_pose(end,:));
            cam_mat1 = cameraMatrix(obj.cameraParams, rotMat1, transVec1);
            
            % Iterate over all candidates
            for i=find(val_cand.')
                % Updating reprojection errors of landmarks
%                 if size(reproError,1)>0
%                     [max_reproError, max_reproError_indx] = max(reproError);
%                     max_reproError_indx = max_reproError_indx(1);
%                 else
%                     max_reproError = Inf;
%                 end

                % Updating tracking score
                if size(curr_tracked_scores,1)>0
                    [min_score, min_score_indx] = min(curr_tracked_scores);
                    min_score_indx = min_score_indx(1);
                else
                    min_score = 0;
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
                    landmarks = [landmarks; cand_landmark]; %#ok<*AGROW>
                    keypoints = [keypoints; candidate_tracked(i,:)];
                    tracking_scores = [tracking_scores; candidate_scores(i)];
                    reproError = [reproError; repro_err];
%                     curr_tracked_scores = [curr_tracked_scores; candidate_scores(i)];
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
            
            if size(landmarks,1) > obj.maxNumLandmarks
                uniformity_scores = uniformityScores(keypoints,...
                    'Sigma', obj.uniformityScoreSigma);
                penalty = (1-obj.penaltyFactor)*(1-uniformity_scores) +...
                    (obj.penaltyFactor)*tracking_scores;
                [~, sort_indcs] = sort(penalty,'descend');
                landmarks = landmarks(sort_indcs(1:obj.maxNumLandmarks),:);
                keypoints = keypoints(sort_indcs(1:obj.maxNumLandmarks),:);
                reproError = reproError(sort_indcs(1:obj.maxNumLandmarks),:);
            end

            curr_state.landmarks = landmarks;
            curr_state.keypoints = keypoints;
            curr_state.reproError = reproError;
            curr_state.candidate_keypoints = candidate_keypoints;
            curr_state.candidate_first_keypoints = candidate_first_keypoints;
            curr_state.candidate_first_poses = candidate_first_poses;
            curr_state.candidate_time_indxs = candidate_time_indxs;            
        end
            
        function [curr_state, curr_pose, pose_status] = processFrame(obj, prev_img, ...
                curr_img, prev_state)
            % PROCESSFRAME: estimates the camera pose and the state given the previous
            % state and the current and previous images
            arguments
                obj
                prev_img % The previous image
                curr_img % The new image
                prev_state % The previous state
            end
            
            %% Estimate camera pose from 2D-3D point correspondences
            fprintf('Iteration\n');
            % Instantiate KLT tracker and initialize
            [tracked_keypoints, val_idx, scores] = obj.tracker.track(prev_img,...
                curr_img, prev_state.keypoints);
            valid_tracked_keypoints = tracked_keypoints(val_idx,:);
            valid_landmarks = prev_state.landmarks(val_idx, :);
            valid_tracked_scores = scores(val_idx,:);
            fprintf('\tPrevious landmarks: \t%d\n',size(tracked_keypoints,1));
            fprintf('\tTracked landmarks: \t%d\n',size(valid_landmarks,1));
            
            % Estimate the pose in world coordinates
            [R_WC, T_WC, inl_idx, pose_status] = estimateWorldCameraPose(...
                double(valid_tracked_keypoints), double(valid_landmarks),...
                obj.cameraParams, ...
                'MaxNumTrials',         obj.ransacMaxNumTrials,...
                'Confidence',           obj.ransacConfidence, ...
                'MaxReprojectionError', obj.ransacMAxReprojectionError);
            
            if pose_status == 2
                warning("Not enough inliers!");
                % FIXME: if this happens, we need more points! Should
                % re-initialize the pipeline?
            end
            
            % If enough inliers were found, run non-linear refinment
            inlier_keypoints = valid_tracked_keypoints(inl_idx,:);
            inlier_landmarks = valid_landmarks(inl_idx,:);

            % Pose non-linear refinement
            intrinsics = obj.getCameraIntrinsics();
            ViewId = uint32(1); AbsolutePose = rigid3d(R_WC, T_WC);
            pointTracks = repmat(pointTrack(1, [0,0]), length(inlier_keypoints),1);
            for i=1:length(inlier_keypoints)
                pointTracks(i).ViewIds = ViewId;
                pointTracks(i).Points = inlier_keypoints(i,:);
            end
            cameraPoses = table(ViewId, AbsolutePose);
            [refined_inlier_landmarks, refinedPoses] =  ...
                bundleAdjustment(inlier_landmarks, pointTracks,...
                cameraPoses, intrinsics);

            % Update pose and landmarks location
            R_WC = refinedPoses.AbsolutePose.Rotation;
            T_WC = refinedPoses.AbsolutePose.Translation;
            valid_landmarks(inl_idx, :) = refined_inlier_landmarks;
            
            % Update the current pose
            curr_pose = [R_WC;T_WC];

            % Discard landmarks that are behind the camera
            [orientation, location] = cameraPoseToExtrinsics(R_WC, T_WC);
            camMat = cameraMatrix(obj.cameraParams, orientation, location);
            ifc_idx = isInFrontOfCamera(camMat, valid_landmarks);
            
            if nnz(~ifc_idx)
                fprintf("Detected landmarks behind the camera! Removing...\n");
            end
            
            % Update kps, landmarks, and tracking scores
            curr_state.keypoints =  valid_tracked_keypoints(ifc_idx,:);
            curr_state.landmarks = valid_landmarks(ifc_idx,:);
            curr_tracked_scores = valid_tracked_scores(ifc_idx,:);
            
            % Update current landmarks' reprojection error
            curr_state.reproError = obj.computeReprojectionError(...
                curr_state.landmarks, curr_state.keypoints, curr_pose);
            
            %% Triangulate new landmarks
            [curr_state, ~] = obj.candidateTriangulation(...
                prev_img, prev_state, curr_img, curr_state, curr_pose, curr_tracked_scores);
            
            %% Select new keypoints to track
            % Only select new keypoints if the number of landmarks which
            % are being tracked is smaller than the allowed maximum
            new_candidate_keypoints = selectCandidateKeypoints(curr_img,...
                [curr_state.keypoints; curr_state.candidate_keypoints],...
                'MinQuality', obj.newCandidateMinQuality, ...
                'FilterSize', obj.newCandidateFilterSize, ...
                'MinDistance', obj.newCandidateMinDistance, ...
                'MaxNewKeypoints', obj.newCandidateMaxNewKeypoints);

            fprintf('\t Curr state fast forwarded candidates: %d\n', size(curr_state.candidate_keypoints,1));
            
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

