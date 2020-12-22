function [landmarks, keypoints, curr_pose] = updateW2D3D(prev_img, curr_img, prev_state, tracker)
    % updates landmarks keypoints and pose from 3D-2D correspondences
    % @param prev_img matrix [NxM] gray scale image of previous frame
    % @param curr_img matrix [NxM] gray scale image of current frame
    % @param prev_state struct previous state structure as defined in
    %        script

    % Track keypoints
    [curr_pts, val_idx, scores] = tracker.track(prev_img,...
                                                curr_img,...
                                                prev_state.keypoints);
    % TODO: check if args in correct format
    [R_WC, T_WC, inl_indx] = estimateWorldCameraPose(curr_pts(val_idx,:),...
                                                     prev_state.landmarks,...
                                                     camera_parameters);

    % Disregarding badly tracked correspondences
    landmarks = prev_state.landmarks(val_idx(inl_indx));
    keypoints = prev_state.keypoints(val_idx(inl_indx));
    
    curr_pose = [R_WC;T_WC];


end

