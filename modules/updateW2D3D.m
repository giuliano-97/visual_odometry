function [landmarks, keypoints, curr_pose] = updateW2D3D(prev_img, curr_img, prev_state, camera_parameters, tracker)
    % updates landmarks keypoints and pose from 3D-2D correspondences
    % @param prev_img matrix [NxM] gray scale image of previous frame
    % @param curr_img matrix [NxM] gray scale image of current frame
    % @param prev_state struct previous state structure as defined in
    %        script

    % Track keypoints
    [curr_pts, val_idx, scores] = tracker.track(prev_img,...
                                                curr_img,...
                                                prev_state.keypoints);
                                            
    prev_img_marked = insertMarker(prev_img,prev_state.keypoints(val_idx,:), 'o','Color','white');
    curr_img_marked = insertMarker(curr_img,curr_pts(val_idx,:), 'o', 'Color', 'white');
            
    % TODO: check if args in correct format
    [R_WC, T_WC, inl_idx] = estimateWorldCameraPose(curr_pts(val_idx,:),...
                                                     prev_state.landmarks(val_idx,:),...
                                                     camera_parameters,...
                                                     'MaxReprojectionError',1,...
                                                     'Confidence', 99,...
                                                     'MaxNumTrials', 10000);
    
    display('Previous Keypoints: '+string(length(val_idx)));
    display('Tracked Keypoints: '+string(sum(val_idx)));
    inl_idx = find(inl_idx);
    val_idx = find(val_idx);
    % Disregarding badly tracked correspondences
    landmarks = prev_state.landmarks(val_idx(inl_idx),:);
    keypoints = curr_pts(val_idx(inl_idx),:);
    
    curr_pose = [R_WC;T_WC];
    
    prev_img_marked = insertMarker(prev_img_marked, prev_state.keypoints(val_idx(inl_idx), :), 'x', 'Color', 'white');
    curr_img_marked = insertMarker(curr_img_marked, keypoints, 'x', 'Color', 'white');
    
    display('Remaining landmarks: '+string(size(landmarks,1)));
    figure
    subplot(1,2,1)
    imshow(prev_img_marked)
    subplot(1,2,2)
    imshow(curr_img_marked)
    

end

