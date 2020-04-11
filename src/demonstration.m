%run('../vlfeat-0.9.21/toolbox/vl_setup.m');
run('vlfeat-0.9.19/toolbox/vl_setup.m');

office = load('data/office1.mat');
office = office.pcl_train;

Translation_array= {} ;
Rotation_array= {} ;
Tform_array = {} ;

for j=1:length(office)-1   
j

pc1 = office{j};
pc2 = office{j+1};

img_ori_1 = imag2d(office{j}.Color);
img_ori_2 = imag2d(office{j+1}.Color);

removeBob = j==27;
removeBob1 = j ==27;
[bin_mask_1, pc1_cleared] = clear_noise(pc1, removeBob);

removeBob = j+1==27;
removeBob2 = j+1==27;
[bin_mask_2, pc2_cleared] = clear_noise(pc2, removeBob);

%% Visualise Original/Masked image and point cloud
%close all;

figure(1);

subplot(2,2,1),
imshow(imag2d(pc1.Color));
title(sprintf('Original 2D Color Image (%dth Frame)',j));

subplot(2,2,2),
indx_xyz_no_1 = find(bin_mask_1==0);
indx_xyz_yes_1 = find(bin_mask_1~=0);
cleared_pc1_color = pc1.Color;
cleared_pc1_color(indx_xyz_no_1, :) = 0;
cleared_pc1_color(indx_xyz_yes_1, :) = 255;
cleared_pc1 = pointCloud(pc1.Location, 'Color', cleared_pc1_color);
imshow(imag2d(cleared_pc1.Color));
title(sprintf('Masked 2D Image (%dth Frame)',j));

subplot(2,2,3),
pcshow(pc1);
xlabel('X');
ylabel('Y');
zlabel('Z');
title(sprintf('Orignal PC (%dth Frame)',j));

subplot(2,2,4),
cleared_pc1_loc = pc1.Location;
cleared_pc1_color = pc1.Color;;
cleared_pc1_color(indx_xyz_no_1, :) = [];
cleared_pc1_loc(indx_xyz_no_1, :) = [];
cleared_pc1 = pointCloud(cleared_pc1_loc, 'Color', cleared_pc1_color);
pcshow(cleared_pc1);
xlabel('X');
ylabel('Y');
zlabel('Z');
title(sprintf('Cleaned PC (%dth Frame)',j));

figure(2);

subplot(2,2,1),
imshow(imag2d(pc2.Color));
title(sprintf('Original 2D Color Image (%dth Frame)',j+1));

subplot(2,2,2),
indx_xyz_no_2 = find(bin_mask_2==0);
indx_xyz_yes_2 = find(bin_mask_2~=0);
cleared_pc2_color = pc2.Color;
cleared_pc2_color(indx_xyz_yes_2, :) = 255;
cleared_pc2_color(indx_xyz_no_2, :) = 0;
cleared_pc2 = pointCloud(pc2.Location, 'Color', cleared_pc2_color);
imshow(imag2d(cleared_pc2.Color));
title(sprintf('Masked 2D Image (%dth Frame)',j+1));

subplot(2,2,3),
pcshow(pc2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title(sprintf('Orignal PC (%dth Frame)',j+1));

subplot(2,2,4),
cleared_pc2_loc = pc2.Location;
cleared_pc2_color = pc2.Color;
cleared_pc2_color(indx_xyz_no_2, :) = [];
cleared_pc2_loc(indx_xyz_no_2, :) = [];
cleared_pc2 = pointCloud(cleared_pc2_loc, 'Color', cleared_pc2_color);
pcshow(cleared_pc2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title(sprintf('Cleaned PC (%dth Frame)',j+1));


%% Extracting frames and descriptors
I1 = single(rgb2gray(img_ori_1));
[f1,d1] = vl_sift(I1,'PeakThresh', 0);
I2 = single(rgb2gray(img_ori_2));
[f2,d2] = vl_sift(I2,'PeakThresh', 0);

% Basic matching
[matches, scores] = vl_ubcmatch(d1,d2,2.5);

% Plot both image side to side
figure(3);
imagesc([img_ori_1, img_ori_2]);
hold on;
[~, offset,~]= size(img_ori_1);

x1s_all = int32(f1(1,:));
y1s_all = int32(f1(2,:));

x2s_all = int32(f2(1,:));
y2s_all = int32(f2(2,:));

[x1s_all, y1s_all, x2s_all, y2s_all] = sift_denoise(bin_mask_1, bin_mask_2, x1s_all, y1s_all, x2s_all, y2s_all);

% Plot all the SIFT points on both images
plot(x1s_all, y1s_all, 'ro', 'MarkerSize', 10);
hold on;
plot(x2s_all+offset, y2s_all, 'ro', 'MarkerSize', 10);
hold on;

x1s = int32(f1(1,(matches(1,:))));
y1s = int32(f1(2,(matches(1,:))));

x2s = int32(f2(1,(matches(2,:))));
y2s = int32(f2(2,(matches(2,:))));

[x1s, y1s, x2s, y2s] = sift_denoise(bin_mask_1, bin_mask_2, x1s, y1s, x2s,y2s);

% Plot matched points in green
hold on;
plot(x1s,y1s,'go','MarkerSize',10);
plot(x2s+offset,y2s,'go','MarkerSize',10);

% Plot the line connecting the SIFT points in two images
hold on;
perm = randperm(length(x1s));
n = min([length(x1s),20]);
sel = perm(1:n); % select 20 random points
for i = 1:length(sel)
    x1 = x1s(sel(i));
    y1 = y1s(sel(i));
    x2 = x2s(sel(i))+offset;
    y2 = y2s(sel(i));
    plot([x1,x2],[y1 y2], 'k', 'LineWidth', 2);
end
title(sprintf('Total number of matched SIFT points = %d',length(x1s)));

%% Convert cleaned siftpoints matches to 3D location
xy_1 = [];
xy_2 = [];

xy_1(:,2) = x1s;
xy_1(:,1) = y1s;

xy_2(:,2) = x2s;
xy_2(:,1) = y2s;

sp_3d_1 = sift_points_3d(pc1.Location, xy_1);
sp_3d_2 = sift_points_3d(pc2.Location, xy_2);

%% REMOVE NAN 
temp_1=sp_3d_1;
temp_2=sp_3d_2;
sp_3d_1(~any((~(isnan(temp_1)|isnan(temp_2))), 2),:)=[];
sp_3d_2(~any((~(isnan(temp_1)|isnan(temp_2))), 2),:)=[];

%% RANSAC
EUCLIDEAN_THRESH = 0.01;
INLIER_THRESH = 0.8;
LIMIT_LOOP = 10000;
MIN_INLIER_COUNT = 50;
NUM_RANSACPOINTS = 25;
INLIERCOUNT_MULT = 0.9;

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

sp_3d_2_pose_estimation = sp_3d_2(sel,:);
sp_3d_2_validation = sp_3d_2;

% Pose Estimation using random sampled points
[est_Translation, est_Rotation] = est_transformation(sp_3d_2_pose_estimation,sp_3d_1_pose_estimation);

% Evaluate new Pc for RANSAC
sp_3d_2_validation_estimate = (est_Rotation*sp_3d_2_validation'+est_Translation)';

euc_dist = sqrt(sum(power(sp_3d_2_validation_estimate-sp_3d_1_validation,2),2));
error = sqrt(sum(sum(power(sp_3d_2_validation_estimate-sp_3d_1_validation,2),2)));

inlier_index = euc_dist <= moving_euc_thres;
inlier_count = sum(inlier_index);
not_enough_points = inlier_count < length(sp_3d_2_validation_estimate)*INLIER_THRESH;

if inlier_count > max_inlier_counts 
    max_inlier_counts = inlier_count;
    best_inlier_index = inlier_index;
    non_improvement_counter = 1;
end

non_improvement_counter = non_improvement_counter + 1;

if non_improvement_counter >= LIMIT_LOOP && max_inlier_counts < moving_min_inlier_count
    not_enough_points = true;
    max_inlier_counts = 0;
    non_improvement_counter = 0;
    moving_min_inlier_count = moving_min_inlier_count*INLIERCOUNT_MULT;
    inlier_count=0;

    moving_min_inlier_count
end
end
max_inlier_counts

[best_est_Translation,best_est_Rotation] = est_transformation(sp_3d_2(find(best_inlier_index==1),:), sp_3d_1(find(best_inlier_index==1),:));

%% Save the best transformation matrix
Translation_array{end+1} = best_est_Translation;
Rotation_array{end+1} = best_est_Rotation;
Tform_array{end+1} = affine3d(horzcat(horzcat(best_est_Rotation, best_est_Translation)',[0 ;0 ;0 ;1]));

%% Visualise Best Post Estimation
new_xyz = pc2_cleared.Location;
new_pc_loc = ( best_est_Rotation*new_xyz'+best_est_Translation )' ;
new_pc = pointCloud( new_pc_loc, 'Color', pc2_cleared.Color );
     
figure(4);
subplot(1,2,1),
pcshowpair(pc1_cleared, pc2_cleared);
title('Before registering')
subplot(1,2,2),
pcshowpair(pc1_cleared,new_pc);
title(sprintf('After registering (using %d pairs of matched points)',sum(best_inlier_index)));

%pause
end
