function [final_mask_bin] = task3_1(pc, removeBob)
%TASK3_1 Summary of this function goes here


%% Task 1, Thresholding 
% Removing outliers data with depth more than a threshold
% Hyperparameter
dept_thres = 3.5;
ori_loc = pc.Location;
ori_color= pc.Color;
z_loc = ori_loc(:,3);
% indx_xyz_no = find(z_loc>3.5 );
indx_xyz_no = find(z_loc>dept_thres | isnan(z_loc));
%indx_xyz_yes = find(z_loc<=3.5);

xyz_pc_1 = ori_loc;
color_pc_1 = ori_color;
%xyz_pc(indx_xyz_no,:) = [];
color_pc_1(indx_xyz_no,:) = 0;
%color_pc(indx_xyz_yes,:) = 255;

% Creating a new point-cloud
new_pc_1 = pointCloud(xyz_pc_1, 'Color', color_pc_1);

%t1_mask = imag2d(new_pc_1.Color);
%t1_mask_bin = t1_mask(:,:,1);
%t1_mask_bin(t1_mask_bin~=0)=255;
bin = color_pc_1~=0;
t1_mask_bin = reshape(bin(:,1), [640, 480]);
final_mask_bin=t1_mask_bin;
% NEW_PC_1 , t1_mask_bin 

%% Task 2, Removing Bob
% Using a hardcoded boundary box to remove Bob
if removeBob%i==27
    %close all;
    obs_mask = ones(640,480);
    obs_mask(83:322,1:480,:)=0;

    t2_mask_bin = bsxfun(@times, final_mask_bin, cast(obs_mask, 'like', final_mask_bin));
    indx_xyz_no = find(t2_mask_bin==0);
    color_pc_2 = ori_color;
    xyz_pc_2 = ori_loc;
    color_pc_2(indx_xyz_no,:) = 0;
    %xyz_pc_2(indx_xyz_no,:) = [];
    new_pc_2 = pointCloud(xyz_pc_2, 'Color', color_pc_2);
    final_mask_bin=t2_mask_bin;
end

%% Task 3, Removing Flying Pixel
% Hyperparameter
num_neighbors = 10; % number of neighbors to be inlier (not flying pixels)
std_dev_thresh = 0.5; % Standard deviation to be considered inliers

color_pc_3 = ori_color;
xyz_pc_3 = ori_loc;

[~, indx_xyz_yes, indx_xyz_no] = pcdenoise(pc,'NumNeighbors',num_neighbors, 'Threshold', std_dev_thresh);
color_pc_3(indx_xyz_no,:) = 0;
new_pc_3 = pointCloud(xyz_pc_3, 'Color' , color_pc_3);

bin_3 = color_pc_3~=0;
t3_mask_bin = reshape(bin_3(:,1), [640, 480]);
t3_mask_bin = bsxfun(@times, final_mask_bin, cast(t3_mask_bin, 'like', final_mask_bin));
final_mask_bin=t3_mask_bin;
%% Task 4, Edge noises
% Hyperparameter 
num_pixel=3; % number of pixel to remove from the edges

t4_mask_bin= ones(640,480);
t4_mask_bin(1:end, 1:num_pixel) = 0;
t4_mask_bin(1:end, end-num_pixel+1:end) = 0;
t4_mask_bin(1:num_pixel, 1:end) = 0;
t4_mask_bin(end-num_pixel+1:end, 1:end) = 0;
t4_mask_bin = bsxfun(@times, final_mask_bin, cast(t4_mask_bin, 'like', final_mask_bin));
final_mask_bin = t4_mask_bin;
end

