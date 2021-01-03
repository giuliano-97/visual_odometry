classdef VOVisualizer < handle
    %VOVISUALIZER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fig
        landmarks
        cameraPose
        scenePlotAxes
        scenePlot
        topViewTrajectory
        topViewTrajectoryPlotRadius
        image
        keypoints
        candidateKeypoints
        numPosesToShow
    end
    
    methods
        function obj = VOVisualizer(optionalArgs)
            arguments
                optionalArgs.trajectoryPlotRadius double = 15
                optionalArgs.numPosesToShow uint32 = 20
            end
            obj.topViewTrajectoryPlotRadius = optionalArgs.trajectoryPlotRadius;
            obj.fig = figure('Name', 'Visual Odometry');
            obj.landmarks = zeros(1,3);
            obj.cameraPose = [eye(3); zeros(1,3)];
            obj.topViewTrajectory = [];
            obj.keypoints = [];
            obj.candidateKeypoints = [];
            obj.numPosesToShow = optionalArgs.numPosesToShow;
        end
        
        function [] = plotTopViewFullTrajectory(obj)
            obj.scenePlotAxes = subplot(2,2,1);
            % Plot the whole trajectory
            plot(obj.topViewTrajectory(:,1),obj.topViewTrajectory(:,2),...
                '-s', 'LineWidth', 1, 'MarkerSize', 2,...
                'MarkerEdgeColor', 'red', 'MarkerFaceColor',[1 .6 .6],...
                'Marker', 'o');
%             if length(obj.topViewTrajectory) > 3
%                 x_min = min(-5, min(obj.topViewTrajectory(:,1)));
%                 x_max = max(5, max(obj.topViewTrajectory(:,1)));
%                 z_min = min(-5, min(obj.topViewTrajectory(:,2)));
%                 z_max = max(5, max(obj.topViewTrajectory(:,2)));
%                 margin_x = 0.1 * (x_max - x_min);
%                 margin_z = 0.1 * (z_max - z_min);
%                 limits_x = [x_min - margin_x, x_max + margin_x];
%                 limits_z = [z_min - margin_z, z_max + margin_z];
%                 xlim(limits_x);
%                 ylim(limits_z);
%             else
%                ylim([-5, 5]); 
%                xlim([-5, 5]);
%             end
            axis('equal');
            xlabel('X');
            ylabel('Z');
            grid on;
        end
        
        function [] = plotTopViewScene(obj)
            subplot(2,2,2);
            hold off;
            % Plot last N poses
            num_poses = length(obj.topViewTrajectory);
            if num_poses > obj.numPosesToShow
                poses_idxs = (num_poses - obj.numPosesToShow):num_poses;
                plot(obj.topViewTrajectory(poses_idxs,1),...
                    obj.topViewTrajectory(poses_idxs,2),...
                    '-s', 'LineWidth', 1, 'MarkerSize', 3,...
                    'MarkerEdgeColor', 'red', 'MarkerFaceColor',[1 .6 .6],...
                    'Marker', 'o');
            else
                plot(obj.topViewTrajectory(:,1),obj.topViewTrajectory(:,2),...
                    '-s', 'LineWidth', 1, 'MarkerSize', 3,...
                    'MarkerEdgeColor', 'red', 'MarkerFaceColor',[1 .6 .6],...
                    'Marker', 'o');
            end
            hold on;
            % Plot the landmarks
            plot(obj.landmarks(:,1), obj.landmarks(:,3), ...
                'LineStyle', 'none', 'Marker', 'diamond', ...
                'MarkerSize', 2, 'MarkerEdgeColor', 'black', ...
                'MarkerFaceColor', 'black');
            xlabel('X');
            ylabel('Z');
            grid on;
        end
        
        function  [] = plotKeypoints(obj, image)
            subplot(2,2,[3,4]);
            if ~isempty(obj.keypoints)
                image = insertMarker(image, obj.keypoints, 'o',...
                    'Size', 6, 'Color', 'green');
            end
            if ~isempty(obj.candidateKeypoints)
                image = insertMarker(image, obj.candidateKeypoints, 'x',...
                    'Size', 6, 'Color', 'red');
            end
            imshow(image);
        end
        
        function [] = update(obj, image, keypoints, ...,
                candidateKeypoints, landmarks, cameraPose)
            % Update data
            obj.image = image;
            obj.keypoints = keypoints;
            obj.candidateKeypoints = candidateKeypoints;
            obj.landmarks = landmarks;
            obj.cameraPose = cameraPose;
            obj.topViewTrajectory = [obj.topViewTrajectory;...
                [cameraPose(4,1), cameraPose(4,3)]];
            % Focus on figure
            figure(obj.fig);
            
            % Plot
            obj.plotTopViewScene();
            obj.plotTopViewFullTrajectory();
            obj.plotKeypoints(image);
        end
    end
end

