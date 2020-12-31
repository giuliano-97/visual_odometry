classdef KLTTracker
% KLTTracker wrappers for matlab KLT Point Tracker
%           Initializes tracker and calculates keypoints
%           in the next frame
    properties
        tracker
    end
    methods
        function obj = KLTTracker(optionalArgs)
            arguments
                optionalArgs.NumPyramidLevels = 7;
                optionalArgs.MaxBidirectionalError = 2;
                optionalArgs.BlockSize = [45 45];
                optionalArgs.MaxIterations = 50;
            end
            obj.tracker = vision.PointTracker(...
                'NumPyramidLevels', optionalArgs.NumPyramidLevels,...
                'MaxBidirectionalError', optionalArgs.MaxBidirectionalError,...
                'BlockSize', optionalArgs.BlockSize,...
                'MaxIterations', optionalArgs.MaxIterations);
        end
        function [points, validity, scores] = track(obj, img1, img2, img1_pts)
            % track Obtains points tracked, validity and scores
            %       of correspondences
            % @param matrix img1:       gray scale image
            % @param matrix img2:       gray scale image
            % @param matrix img1_pts:   [N,2] float of detections in img1
            % @output matrix points:    [N,2] float of points tracked form
            %                           img1 to img2
            % @output matrix validity   [N,1] bool representing validity of
            %                           tracked points
            % @output matrix score      [N,1] float scores for each track
            points = double.empty(0,2);
            validity = [];
            scores = double.empty(0,1);
            if size(img1_pts,1)>0
                obj.tracker.release();
                initialize(obj.tracker, img1_pts, img1);
                [points, validity, scores] = obj.tracker(img2);
            end
        end
    end
end