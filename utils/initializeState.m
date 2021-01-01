function state = initializeState(landmarks, keypoints, reproError, pose, time_indx)
%INITIALIZESTATE Initializes state for Visual Odometry
%   @param landmarks matrix [Nx3]
%   @param keypoints matrix [NX2]
%   @param time_indx int>0          Time indicator of 
state.landmarks = landmarks;
state.keypoints = keypoints;
state.reproError = reproError;
state.candidate_keypoints = keypoints;
state.candidate_first_keypoints = keypoints;
state.candidate_first_poses = [{}];
state.candidate_first_poses = [state.candidate_first_poses, repmat({pose},1,size(keypoints,1))];
state.candidate_time_indxs = repmat(-time_indx, 1,size(keypoints,1));
end

