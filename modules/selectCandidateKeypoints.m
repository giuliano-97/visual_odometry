function validIndex = selectCandidateKeypoints(kp0_pos, kp1_pos, ...
    optionalArgs)
% SELECTCANDIDATEKEYPOINTS Selects a subset of new candidate keypoints
% For each new keypoint, this function checks whether there is already
% a keypoint with a given distance. If not, the keypoint is selected as 
% candidate for triangulation
% 
%   validIndex = selectCandidateKeypoints(kp0_pos, kp1_pos) returns a
%   logical array indicating which newly extract keypoints out of kp1_pos
%   can be used as candidates for traingulation.
arguments
   kp0_pos   % Image coordinates of the tracked keypoints
   kp1_pos   % Image coordinates of the new keypoints
   optionalArgs.MinDistance double = 10 % The (optional) distance threshold
end

% Compute the distance of all new keypoints from all the old keypoints
D = pdist2(kp1_pos, kp0_pos, 'euclidean');

% Find all the distances smaller than the threshold
L = D < optionalArgs.MinDistance;

% Compute the valid index - ith entry is false if there is already a
% keypoints within an area of radius R from the ith new keypoint
validIndex = sum(L,2) == 0;

end

