function new_kps_loc = selectCandidateKeypoints(img, curr_kps, optionalArgs)
% SELECTCANDIDATEKEYPOINTS Selects a subset of new candidate keypoints
% For each new keypoint, this function checks whether there is already
% a keypoint with a given distance. If not, the keypoint is selected as 
% candidate for triangulation
% 
%   validIndex = selectCandidateKeypoints(kp0_pos, kp1_pos) returns a
%   logical array indicating which newly extract keypoints out of kp1_pos
%   can be used as candidates for traingulation.
arguments
   img % The image in which the new keypoints should be extracted
   curr_kps   % Image coordinates of the existing tracked and candidate keypoints
   optionalArgs.MinDistance double = 20 % The (optional) distance threshold
   optionalArgs.MinQuality double = 0.01 % Harris keypoints quality
   optionalArgs.FilterSize double = 3 % Harris detector filter size
   optionalArgs.CandidatesToKeep double = 200 % Detected valid kps to keep
end

% Detect new keypoints
new_kps = detectMinEigenFeatures(img,'MinQuality', optionalArgs.MinQuality);

% Compute the distance of all new keypoints from all existing keypoints
D = pdist2(single(new_kps.Location), single(curr_kps), 'euclidean');

% Find all the distances smaller than the threshold
L = D < optionalArgs.MinDistance;

% Keep only the keypoints that are far enough from the existing ones
new_kps = new_kps(sum(L,2) == 0);

% Cap the maximum number of keypoints - pick the required number uniformly
new_kps = selectUniform(new_kps, optionalArgs.CandidatesToKeep, size(img));

new_kps_loc = new_kps.Location;
end

