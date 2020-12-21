function validIndex = selectCandidateKeypoints(keypoints0,keypoints1,...
    optionalArgs)
% SELECTCANDIDATEKEYPOINTS Selects a subset of new candidate keypoints
% For each new keypoint, this function checks whether there is already
% a keypoint with a given distance. If not, the keypoint is selected as 
% candidate for triangulation
arguments
   keypoints0   % The set of existing keypoints
   keypoints1   % The set of new keypoints
   optionalArgs.MinDistance double = 10 % The (optional) distance threshold
end

% Fetch the keypoint locations
keypoints0_coords = keypoints0.Location;
keypoints1_coords = keypoints1.Location;

% Compute the distance of all new keypoints from all the old keypoints
D = pdist2(keypoints1_coords, keypoints0_coords, 'seuclidean');

% Find all the distances smaller than the threshold
L = D < optionalArgs.MinDistance;

% Compute the valid index - ith entry is true if there is already a
% keypoints within an area of radius R from the ith new keypoint
validIndex = sum(L,2) > 0;

end

