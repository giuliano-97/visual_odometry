function state = initializeState(landmarks, keypoints, reproError, pose, time_indx)
%INITIALIZESTATE Initializes state for Visual Odometry
%   @param landmarks matrix [Nx3]
%   @param keypoints matrix [NX2]
%   @param time_indx int>0          Time indicator of 
state.landmarks = landmarks;
state.keypoints = keypoints;
state.reproError = reproError;
state.candidate_keypoints = [];
state.candidate_first_keypoints = [];
state.candidate_first_poses = [];
state.candidate_time_indxs = [];
end

