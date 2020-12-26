function [camParams, ground_truth, last_frame] = loadGeneralData(ds)
% loadGeneralData: loads data descriptive of the dataset
% @param ds int 0: KITTI, 1: Malaga, 2: parking

    % Datasets paths
    kitti_path = 'data/kitti';
    parking_path = 'data/parking';
    malaga_path = 'data/malaga-urban-dataset-extract-07';

    if ds == 0
        % need to set kitti_path to folder containing "00" and "poses"
        assert(exist('kitti_path', 'var') ~= 0);
        ground_truth = load([kitti_path '/poses/00.txt']);
        ground_truth = ground_truth(:, [end-8 end]);
        last_frame = 4540;
        K = [7.188560000000e+02 0 6.071928000000e+02
            0 7.188560000000e+02 1.852157000000e+02
            0 0 1];
    elseif ds == 1
        % Path containing the many files of Malaga 7.
        assert(exist('malaga_path', 'var') ~= 0);
        images = dir([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        last_frame = length(left_images);
        K = [621.18428 0 404.0076
            0 621.18428 309.05989
            0 0 1];
        ground_truth = [];
    elseif ds == 2
        % Path containing images, depths and all...
        assert(exist('parking_path', 'var') ~= 0);
        last_frame = 598;
        K = load([parking_path '/K.txt']);

        ground_truth = load([parking_path '/poses.txt']);
        ground_truth = ground_truth(:, [end-8 end]);
    else
        assert(false);
    end

    % Create cameraParameters object using the K matrix
    camParams = cameraParameters('IntrinsicMatrix', K.');
end

