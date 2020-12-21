function [curr_state, curr_pose] = processFrame(prev_state, prev_img,...
    curr_img, camera_matrix)
% processFrame: returns state and camera pose at the current frame given
% the state and the image at the previous frame and the new image
% @param struct prev_state: The state at the previous frame
% @param matrix prev_img: Image at the previous frame
% @param matrix curr_img: Image at the current frame
% @return struct curr_state: The new state
% @return matrix curr_pose: The new pose (TODO: add description)

    persistent tracker
    persistent K
    if isempty(tracker)
        tracker = KLTTracker();
    end
    if nargin > 2
        K = camera_matrix;
    end
    
    
    admissible_angular_threshold = 5;
    max_temporal_recall = 20;
    
    curr_state.landmarks = landmarks;
    curr_state.keypoints = keypoints;
    curr_state.candidate_keypoints = [];
    curr_state.candidate_first_keypoints = [];
    curr_state.candidate_first_poses = [];
    curr_state.candidate_time_indxs = [];
    
    % Track keypoints
    [curr_pts, val_idx, scores] = tracker.track(prev_img,...
                                                curr_img,...
                                                prev_state.keypoints);
    % TODO: check if args in correct format
    [R_WC, T_WC, inl_indx] = estimateWorldCameraPose(curr_pts(val_idx,:),...
                                                     prev_state.landmarks,...
                                                     K);

    % Disregarding badly tracked correspondences
    curr_state.landmarks = prev_state.landmarks(val_idx(inl_indx));
    curr_state.keypoints = prev_state.keypoints(val_idx(inl_indx));
    
    curr_pose = [R_WC;T_WC];
    
    
    %% Introducing candidates if appropriate
    [candidate_tracked, val_cand, score] =tracker.track(prev_img,...
                                          curr_img,...
                                          prev_state.candidate_keypoints);
    for i=val_cand
        cand_landmark = triangulate(prev_state.candidate_first_keypoints(i,:),...
                                    candidate_tracked(i,:),...
                                    prev_state.candidate_first_poses{i},...
                                    curr_pose);
                                
        % TODO: define calculate_angle function
        if calculate_angle(cand_landmark, prev_state.candidate_first_poses{i},...
                            curr_pose) > admissible_angular_threshold
            curr_state.landmarks = [curr_state.landmarks;...
                                    cand_landmark];
            curr_state.keypoints = [curr_state.keypoints;...
                                    candidate_tracked(i,:)];
        else
            % Discard candidate if has been stored for too long
            if prev_state.candidate_time_indxs(i) >= -max_temporal_recall
                curr_state.candidate_keypoints = [curr_state.dandidate_keypoints;...
                                                  candidate_tracked(i,:)];
                curr_state.candidate_first_keypoints = [curr_state.candidate_first_keypoints(i,:);...
                                                        prev_state.candidate_first_keypoints(i,:)];
                curr_state.candidate_first_poses = [curr_state.candidate_first_poses(i,:);...
                                                     prev_state.candidate_first_poses(i)];
                curr_state.cadidate_time_indxs = [curr_state.candidate_time_indxs, ...
                                                  prev_state.candidate_time_indxs(i)-1];
            end
        end
        
    end
    
    
    % TODO: Detect new features in current frame and add them to candidates
    % if not already candidates
    
    
end

