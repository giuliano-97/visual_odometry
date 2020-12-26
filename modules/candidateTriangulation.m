function [curr_state, tracked_keypoints] = candidateTriangulation(prev_img,...
    prev_state, curr_img, curr_pose, camera_params,tracker,optionalArgs)
    arguments
        prev_img
        prev_state
        curr_img
        curr_pose
        camera_params
        tracker
        optionalArgs.max_temporal_recall = 50;
        optionalArgs.admissible_angular_threshold = 5;
    end
    
    %% Tracking current keypoints and landmarks
    [keypoints_tracked, val_key, ~] = tracker.track(prev_img,...
                                        curr_img,...
                                        prev_state.keypoints);
     landmarks = prev_state.landmarks(val_key, :);
     keypoints = keypoints_tracked(val_key, :);
    
    %% Introducing candidates if appropriate
    [candidate_tracked, val_cand, score] =tracker.track(prev_img,...
                                          curr_img,...
                                          prev_state.candidate_keypoints);
    tracked_keypoints = candidate_tracked(val_cand,:);
    
    candidate_keypoints = double.empty(0,2);
    candidate_first_keypoints = double.empty(0,2);
    candidate_first_poses = [{}];
    candidate_time_indxs = [];
    
    for i=find(val_cand.')
        [rotMat0, transVec0] = cameraPoseToExtrinsics(...
            prev_state.candidate_first_poses{i}(1:3,:),...
            prev_state.candidate_first_poses{i}(end,:));
        cam_mat0 = cameraMatrix(camera_params, rotMat0, transVec0);
        [rotMat1, transVec1] = cameraPoseToExtrinsics(...
            curr_pose(1:3,:),...
            curr_pose(end,:));
        cam_mat1 = cameraMatrix(camera_params, rotMat1, transVec1);
        
        cand_landmark = triangulate(prev_state.candidate_first_keypoints(i,:),...
                                    candidate_tracked(i,:),...
                                    cam_mat0,...
                                    cam_mat1);
        
        % Add views with keypoints to viewset
%         vSet = imageviewset;
%         vSet = addView(vSet, 1, rigid3d(...
%             [prev_state.candidate_first_poses{i};[0;0;0;1]]),...
%             'Points', prev_stat.candidate_tracked(i,:));
%         vSet = addView(vSet, 2, rigid3d(R1, t1), 'Points',...
%             matched_points1_inliers);
% 
%         % Add correspondences to viewset
%         vSet = addConnection(vSet, 1, 2, 'Matches', ...
%             [1:num_inliers; 1:num_inliers]');
% 
%         % Find tracks
%         tracks = findTracks(vSet);
% 
%         % Get camera poses
%         cameraPoses = poses(vSet);
% 
%         % Triangulate points
%         [landmarks, ~, validIndex] = triangulateMultiview(tracks, ...
%             cameraPoses, intrinsics);
                                
                                
        % TODO: define calculate_angle function
        if calculateAngleDeg(cand_landmark, prev_state.candidate_first_poses{i},...
                            curr_pose) > optionalArgs.admissible_angular_threshold
            landmarks = [landmarks; cand_landmark];
            keypoints = [keypoints; candidate_tracked(i,:)];
        else
            % Discard candidate if has been stored for too long
            if prev_state.candidate_time_indxs(i) > -optionalArgs.max_temporal_recall
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
