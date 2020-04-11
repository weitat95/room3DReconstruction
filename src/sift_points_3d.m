function [output]= sift_points_3d(xyz_loc, xy_index)
% xyz_loc   : Location from point cloud
% xy_index  : X,Y index for the matched sift points of size (number of matched points x 2)
xy_1=xy_index;
xyz_1 = xyz_loc;

% reshape the location 
rec_x = reshape(xyz_1(:,1), [640,480]);
rec_y = reshape(xyz_1(:,2), [640,480]);
rec_z = reshape(xyz_1(:,3), [640,480]);
xyz_1_reshape = cat(3, rec_x', rec_y', rec_z'); 

points_frame1=[];
for i = 1:size(xy_index,1)
    points_frame1 = [points_frame1; reshape(xyz_1_reshape(xy_1(i,1), xy_1(i,2), :), [1,3])];
end
output = points_frame1;
end

