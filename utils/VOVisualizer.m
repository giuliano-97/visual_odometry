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
    end
    
    methods
        function obj = VOVisualizer(optionalArgs)
            arguments
               optionalArgs.trajectoryPlotRadius double = 15; 
            end
            obj.topViewTrajectoryPlotRadius = optionalArgs.trajectoryPlotRadius;
            obj.fig = figure('Name', 'Visual Odometry');
            obj.landmarks = zeros(1,3);
            obj.cameraPose = [eye(3); zeros(1,3)];
            obj.topViewTrajectory = [];
            obj.keypoints = [];
            obj.candidateKeypoints = [];
        end
        
        function [] = plotScene(obj)
            obj.scenePlotAxes = subplot(2,2,1);
            % Plot the current point cloud
            hold off
            plot3(obj.landmarks(:,1), obj.landmarks(:,2), obj.landmarks(:,3), '*'); 
            hold on;
            % Plot the current camera pose
            orientation = obj.cameraPose(1:3,1:3);
            position = obj.cameraPose(4,:);
            absPose = rigid3d(orientation, position);
            plotCamera('AbsolutePose', absPose, 'Size', 1);
            hold on;
            set(gca, 'CameraUpVector', [0 1 0]);
            grid on;
            xlim([position(1) - 20, position(1) + 20]);
            ylim([position(2) - 20, position(2) + 20]);
            zlim([position(3) - 10, position(3) + 50]);
            
            % Make sure figure background remains white after pcshow
            obj.fig.Color = 'white';
        end
        
        function [] = plotTopViewTrajectory(obj)
            subplot(2,2,2);
            plot(obj.topViewTrajectory(:,1),obj.topViewTrajectory(:,2),...
                '-s', 'LineWidth', 1, 'MarkerSize',1,...
                'MarkerEdgeColor','red', 'MarkerFaceColor',[1 .6 .6],...
                'Marker', 'o');
            xlim([obj.cameraPose(4,1) - obj.topViewTrajectoryPlotRadius,...
                obj.cameraPose(4,1) + obj.topViewTrajectoryPlotRadius]);
            ylim([obj.cameraPose(4,3) - obj.topViewTrajectoryPlotRadius,...
                obj.cameraPose(4,3) + obj.topViewTrajectoryPlotRadius]);
            xlabel('X');
            ylabel('Z');
            grid on;
        end
        
        function  [] = plotKeypoints(obj, image)
            subplot(2,2,[3,4]);
            if ~isempty(obj.keypoints)
                image = insertMarker(image, obj.keypoints, 'x',...
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
            obj.plotScene();
            obj.plotTopViewTrajectory();
            obj.plotKeypoints(image);
        end
        
        
    end
end

