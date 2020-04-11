function [best_est_Translation, best_est_Rotation, last_min_error] = pose_estimation(pc1, pc2, clearNoise, removeBob)
%POSE_ESTIMATION Summary of this function goes here

img_ori_1 = imag2d(pc1.Color);
img_ori_2 = imag2d(pc2.Color);

%% Extracting frames and descriptors
I1 = single(rgb2gray(img_ori_1));
[f1,d1] = vl_sift(I1, 'PeakThresh', 0);
I2 = single(rgb2gray(img_ori_2));
[f2,d2] = vl_sift(I2, 'PeakThresh', 0);


%% Matching
[matches, scores] = vl_ubcmatch(d1,d2,2.5);

%% Extract Sift Point Matches in 3D coordinates;
xyz_1 = pc1.Location;
xyz_2 = pc2.Location;

xy_1 = [];
xy_2 = [];

xy_1(:,2) = int32(f1(1,(matches(1,:))))';
xy_1(:,1) = int32(f1(2,(matches(1,:))))';

xy_2(:,2) = int32(f2(1,(matches(2,:))))';
xy_2(:,1) = int32(f2(2,(matches(2,:))))';

%sp_3d_1 = sift_points_3d(xyz_1, xy_1);
%sp_3d_2 = sift_points_3d(xyz_2, xy_2);

if clearNoise==true 
    [bin_mask_1, pc1_cleared] = clear_noise(pc1, removeBob(1));
    [bin_mask_2, pc2_cleared] = clear_noise(pc2, removeBob(2));

    [r1,c1] = find(bin_mask_1'==0);
    [r2,c2] = find(bin_mask_2'==0);
    counter=0;
    for i=1:length(xy_1)

        if(sum(sum(xy_1(i,:) == [c1,r1],2)==2)~=0)
            counter = counter+1;
            xy_1(i,:)=[-1,-1];
            xy_2(i,:)=[-1,-1];
        end

    end
    for i=1:length(xy_2)
        if(sum(sum(xy_2(i,:) == [c2,r2],2)==2)~=0)
            counter = counter+1;
            xy_2(i,:)=[-1, -1];
            xy_1(i,:)=[-1, -1];
        end
    end
    xy_1(xy_1(:,1)==-1,:) = [];
    xy_2(xy_2(:,1)==-1,:) = [];

end
length(xy_1)
length(xy_2)
%% Convert siftpoints matches to 3d 
xyz_1 = pc1.Location;
xyz_2 = pc2.Location;

sp_3d_1 = sift_points_3d(xyz_1, xy_1);
sp_3d_2 = sift_points_3d(xyz_2, xy_2);

%% REMOVE NAN 
temp_1=sp_3d_1;
temp_2=sp_3d_2;
sp_3d_1(~any((~(isnan(temp_1)|isnan(temp_2))), 2),:)=[];
sp_3d_2(~any((~(isnan(temp_1)|isnan(temp_2))), 2),:)=[];

if clearNoise==true 

    pc1 = pc1_cleared;
    pc2 = pc2_cleared;
end

%% Ransac
EUCLIDEAN_THRESH = 0.01;
INLIER_THRESH = 0.8;
LIMIT_LOOP = 10000;
MIN_INLIER_COUNT = 50;
EUCL_MULT = 1.5;
NUM_RANSACPOINTS = 25;
INLIERCOUNT_MULT = 0.9;

last_min_error = 9999999999 ;
not_enough_points = true;
max_inlier_counts = 0;
non_improvement_counter = 0;
moving_euc_thres = EUCLIDEAN_THRESH;
moving_min_inlier_count = MIN_INLIER_COUNT;
size(sp_3d_1,1)
while not_enough_points && non_improvement_counter < LIMIT_LOOP
    
n =min([NUM_RANSACPOINTS, size(sp_3d_1)]); % select 15 random points

perm = randperm(size(sp_3d_1,1));
sel = perm(1:n); 

sp_3d_1_pose_estimation = sp_3d_1(sel,:);
sp_3d_1_validation = sp_3d_1;
%sp_3d_1_validation(sel,:)=[];

sp_3d_2_pose_estimation = sp_3d_2(sel,:);
sp_3d_2_validation = sp_3d_2;
%sp_3d_2_validation(sel,:)=[];

% To plot line to visualize matching in 3D
% new_xyz = office{2}.Location;
% newpc = pointCloud(new_xyz + [ 3 , 0, 0],'Color', office{2}.Color);
% pcshow(office{1}), hold on, pcshow(newpc);
% pts = [ points_frame1(1,:); points_frame2(1,:)+[3 0 0] ];
% hold on;
% plot3(pts(:,1),pts(:,2),pts(:,3), 'LineWidth',50);



%% Pose Estimation using random sampled points
[est_Translation, est_Rotation] = est_transformation(sp_3d_1_pose_estimation,sp_3d_2_pose_estimation);

%% Evaluate new Pc for RANSAC
sp_3d_1_validation_estimate = (est_Rotation*sp_3d_1_validation'+est_Translation)';

euc_dist = sqrt(sum(power(sp_3d_1_validation_estimate-sp_3d_2_validation,2),2));
error = sqrt(sum(sum(power(sp_3d_1_validation_estimate-sp_3d_2_validation,2),2)));

inlier_index = euc_dist <= moving_euc_thres;
inlier_count = sum(inlier_index);
not_enough_points = inlier_count < length(sp_3d_1_validation_estimate)*INLIER_THRESH;


if inlier_count > max_inlier_counts 
    max_inlier_counts = inlier_count;
    best_inlier_index = inlier_index;
    non_improvement_counter = 1;
    %max_inlier_counts
end
%     if (error < last_min_error - threshold) 
%         improvement_since=0;
%         last_min_error = error;
%         best_est_Rotation = est_Rotation;
%         best_est_Translation = est_Translation;
%     else
%         %no improvement
%         improvement_since = improvement_since + 1;
%     end
non_improvement_counter = non_improvement_counter + 1;
%non_improvement_counter
if non_improvement_counter >= LIMIT_LOOP && max_inlier_counts < moving_min_inlier_count
    %moving_euc_thres = moving_euc_thres*EUCL_MULT;
    not_enough_points = true;
    max_inlier_counts = 0;
    non_improvement_counter = 0;
    moving_min_inlier_count = moving_min_inlier_count*INLIERCOUNT_MULT;
    inlier_count=0;

    moving_min_inlier_count
    %moving_euc_thres
end
end
max_inlier_counts
last_min_error = error;

% Best Pose Estimation using all inlier points
[best_est_Translation,best_est_Rotation] = est_transformation(sp_3d_1(find(best_inlier_index==1),:), sp_3d_2(find(best_inlier_index==1),:));
sum(best_inlier_index)
% last_min_error
% 
% %% Visualise Best Pose Estimation 
% close all;
% new_xyz = pc1.Location;
% new_pc_loc = (best_est_Rotation*new_xyz'+best_est_Translation)' ;
% new_pc = pointCloud(new_pc_loc, 'Color', pc1.Color);
% subplot(1,2,1), pcshow(pc1), hold on, pcshow(pc2);
% subplot(1,2,2), pcshow(new_pc), hold on, pcshow(pc2);
% 
end

