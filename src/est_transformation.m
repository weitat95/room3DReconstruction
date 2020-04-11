function [est_Translation,est_Rotation] = est_transformation(xyz_3d_1, xyz_3d_2)

sp_3d_1_pose_estimation = xyz_3d_1;
sp_3d_2_pose_estimation = xyz_3d_2;

%% Pose Estimation using SVD of a matrix
% Finding Centroid
centroid_1 = sum(sp_3d_1_pose_estimation)/length(sp_3d_1_pose_estimation);
centroid_2 = sum(sp_3d_2_pose_estimation)/length(sp_3d_2_pose_estimation);

% Centre point set
centered_1 = sp_3d_1_pose_estimation-centroid_1;
centered_2 = sp_3d_2_pose_estimation-centroid_2;

% Correlation Matrix H
H = centered_1'* centered_2; % 3x3 Matrix

% SVD 
[U, ~, V] = svd(H); 

est_Rotation = V*U';
if cast(det(est_Rotation), 'int32') == -1
     V(:,3)=-V(:,3);
     est_Rotation = V*U';
end
est_Translation = centroid_2' - est_Rotation*centroid_1';

end

