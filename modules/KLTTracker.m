classdef KLTTracker
% KLTTracker wrappers for matlab KLT Point Tracker
%           Initializes tracker and calculates keypoints
%           in the next frame
    properties
        tracker
    end
    methods
        function obj = KLTTracker(varargin)
            % varargins include:
            %   'NumPyramidLevels'      =   3
            %   'MaxBidirectionalError' =   Inf (default)
            %   'BlockSize'             =   [31 31]
            %   'MaxIterations'         =   30
      
            obj.tracker = vision.PointTracker(varargin{:});
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
            
            initialize(obj.tracker, img1_pts, img1);
            [points, validity, scores] = obj.tracker(img2);
        end
    end
end