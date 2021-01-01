function valid_idx = isInFrontOfCamera(camMat, points3d)
%ISINFRONTOFCAMERA Verify if points3d are in front of the camera
    arguments
        % TODO: argument type and size validation
        camMat
        points3d
    end
camMat = cast(camMat, 'like', points3d)';
points3dHomog = [points3d, ones(size(points3d, 1), 1, 'like', points3d)]';
points2dHomog = camMat * points3dHomog;
valid_idx = points2dHomog(3, :)' > 0;
end