classdef KLTTracker
    properties
        tracker
    end
    methods
        function obj = KLTTracker(pyramid_lvls,...
                bidirectional_err,...
                block_size, ...
                max_iter)
            obj.tracker = vision.PointTracker('NumPyramidLevels', pyramid_lvls,...
                                  'MaxBidirectionalError', bidirectional_err,...
                                  'BlockSize',block_size,...
                                  'MaxIterations',max_iter);
        end
        function [points, validity, scores] = track(obj, img1, img2, img1_pts)
            initialize(obj.tracker, img1_pts, img1);
            [points, validity, scores] = obj.tracker(img2);
        end
    end
end