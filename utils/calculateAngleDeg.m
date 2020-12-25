function angle = calculateAngleDeg(landmark, cam_pose0, cam_pose1)
    % calculateAngleDeg calculates angle between projection rays
    % @param landmark  matrix [1x3] 3D point projected in to camera frames
    % @param cam_pose0 matrix [4x3] pose of camera1
    % @param cam_pose1 matirx [4x3] pose of camera2
    
    vec0 = landmark - cam_pose0(end,:);
    vec1 = landmark - cam_pose1(end,:);
    cosTheta = dot(vec0,vec1)/(norm(vec0)*norm(vec1));
    angle = acosd(cosTheta);
end
