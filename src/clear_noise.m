function [final_mask_bin, pc_cleared] = clear_noise(pc, removeBob)
%TASK3_1 Summary of this function goes here
ori_loc = pc.Location;
ori_color= pc.Color;
final_mask_bin= ones(640,480);
TASK1=true;
TASK2=true;
TASK3=true;
TASK4=true;

% Task 1, Thresholding 
%Removing outliers data with depth more than a threshold
%Hyperparameter
if( TASK1 )
dept_thres = 3.5;

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
%NEW_PC_1 , t1_mask_bin 
end
%% Task 2, Removing Bob
% Using a hardcoded boundary box to remove Bob
if (TASK2)
if removeBob%i==27
    %close all;
    % HEAD
    % X- (192,270)
    % Y- (1, 85)
    % Body
    % X- (83,322)
    % Y- (85, 480)
    obs_head = ones(640,480);
    obs_head(192:270,1:85,:)=0;
    obs_body = ones(640,480);
    obs_body(83:322, 85:480)=0;     
%     obs_mask = ones(640,480);
%     obs_mask(83:322,1:480,:)=0;
    t2_mask_bin = bsxfun(@times, final_mask_bin, cast(obs_head, 'like', final_mask_bin));
    t2_mask_bin = bsxfun(@times, t2_mask_bin, cast(obs_body, 'like', t2_mask_bin));
    %t2_mask_bin = bsxfun(@times, final_mask_bin, cast(obs_mask, 'like', final_mask_bin));
    indx_xyz_no = find(t2_mask_bin==0);
    color_pc_2 = ori_color;
    xyz_pc_2 = ori_loc;
    color_pc_2(indx_xyz_no,:) = 0;
    %xyz_pc_2(indx_xyz_no,:) = [];
    new_pc_2 = pointCloud(xyz_pc_2, 'Color', color_pc_2);
    final_mask_bin=t2_mask_bin;
end
end
% %% Task 3, Removing Flying Pixel
% % Hyperparameter
if (TASK3)
K = 100;
DIST_THRESH = 0.02;
INLIER_THRESH = 22;
color_pc_3 = ori_color;
xyz_pc_3 = ori_loc;

indx_xyz_no = [];
% for i = 1 : length(xyz_pc_3)
%     i
%     if (isnan(xyz_pc_3(i,1)))
%        continue; 
%     end
%     [ ~, nearest_dist ] =findNearestNeighbors(pc, xyz_pc_3(i,:), K); 
%     if (sum(nearest_dist<DIST_THRESH) < INLIER_THRESH)
%         indx_xyz_no = [indx_xyz_no ; i];
%     end
% end

num_neighbors = 30; % number of neighbors to be inlier (not flying pixels)
dist_thres = 0.05; % Standard deviation to be considered inliers

color_pc_3 = ori_color;
xyz_pc_3 = ori_loc;

%[~, indx_xyz_yes, indx_xyz_no] = pcdenoise(pc,'NumNeighbors',num_neighbors, 'Threshold', std_dev_thresh);

[~, dist_3] = knnsearch(xyz_pc_3, xyz_pc_3,'K',num_neighbors);
indx_xyz_no = find((max(dist_3, [], 2)-min(dist_3, [] ,2) > dist_thres)==1);
%find(dist_3 < 1.0);
color_pc_3(indx_xyz_no,:) = 0;
%new_pc_3 = pointCloud(xyz_pc_3, 'Color' , color_pc_3);

bin_3 = color_pc_3~=0;
t3_mask_bin = reshape(bin_3(:,1), [640, 480]);
t3_mask_bin = bsxfun(@times, final_mask_bin, cast(t3_mask_bin, 'like', final_mask_bin));
final_mask_bin=t3_mask_bin;
end
if (TASK4)
%% Task 4, Edge noises
% Hyperparameter 
num_pixel=10; % number of pixel to remove from the edges

t4_mask_bin= ones(640,480);
t4_mask_bin(1:end, 1:num_pixel) = 0;
t4_mask_bin(1:end, end-num_pixel+1:end) = 0;
t4_mask_bin(1:num_pixel, 1:end) = 0;
t4_mask_bin(end-num_pixel+1:end, 1:end) = 0;
t4_mask_bin = bsxfun(@times, final_mask_bin, cast(t4_mask_bin, 'like', final_mask_bin));
final_mask_bin = t4_mask_bin;
end
%% Output cleared pc
indx_xyz_no = find(final_mask_bin==0);
xyz_pc_temp = ori_loc;
color_pc_temp = ori_color;

xyz_pc_temp(indx_xyz_no, :) = [];
color_pc_temp(indx_xyz_no, :) = [];

pc_cleared = pointCloud(xyz_pc_temp, 'Color', color_pc_temp);
end

