office = load('data/office1.mat');
office = office.pcl_train;

pc1 = office{1};
pc2 = office{2};

img_ori_1 = imag2d(pc1.Color);
img_ori_2 = imag2d(pc2.Color);

%% Extracting frames and descriptors
I = single(rgb2gray(img_ori_1));
[f,d] = vl_sift(I);
I2 = single(rgb2gray(img_ori_2));
[f2,d2] = vl_sift(I2);

%% Matching
[matches, scores] = vl_ubcmatch(d,d2);

%% Extract Sift Point Matches in 3D coordinates;
xy_1 = [];
xy_2 = [];

xy_1(:,2) = int32(f(1,(matches(1,:))))';
xy_1(:,1) = int32(f(2,(matches(1,:))))';

xy_2(:,2) = int32(f2(1,(matches(2,:))))';
xy_2(:,1) = int32(f2(2,(matches(2,:))))';


%% Clear Noise from Task 3.1
clearNoise=true;
if clearNoise==true 
    [bin_mask_1, pc1_cleared] = clear_noise(pc1, false);
    [bin_mask_2, pc2_cleared] = clear_noise(pc2, false);

    [r1,c1] = find(bin_mask_1==0);
    [r2,c2] = find(bin_mask_2==0);
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

    display("CLEAR NOISE");
    display(counter);
end
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
improvement_since = 0 ;
last_min_error = 9999999999 ;
threshold = 0.01 ;
while improvement_since < 6
    
n = 15; % select 15 random points
perm = randperm(size(sp_3d_1,1));
sel = perm(1:n); 

sp_3d_1_pose_estimation = sp_3d_1(sel,:);
sp_3d_1_validation = sp_3d_1;
sp_3d_1_validation(sel,:)=[];

sp_3d_2_pose_estimation = sp_3d_2(sel,:);
sp_3d_2_validation = sp_3d_2;
sp_3d_2_validation(sel,:)=[];

% To plot line to visualize matching in 3D
% new_xyz = office{2}.Location;
% newpc = pointCloud(new_xyz + [ 3 , 0, 0],'Color', office{2}.Color);
% pcshow(office{1}), hold on, pcshow(newpc);
% pts = [ points_frame1(1,:); points_frame2(1,:)+[3 0 0] ];
% hold on;
% plot3(pts(:,1),pts(:,2),pts(:,3), 'LineWidth',50);



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
est_Translation = centroid_2' - est_Rotation*centroid_1';


%% Evaluate new Pc for RANSAC
sp_3d_1_validation_estimate = (est_Rotation*sp_3d_1_validation'+est_Translation)';

error = sqrt(sum(sum(power(sp_3d_1_validation_estimate-sp_3d_2_validation,2),2)));

    if (error < last_min_error - threshold) 
        improvement_since=0;
        last_min_error = error;
        best_est_Rotation = est_Rotation;
        best_est_Translation = est_Translation;
    else
        %no improvement
        improvement_since = improvement_since + 1;
    end
end
last_min_error



%% Visualise Best Pose Estimation 
close all;
new_xyz = office{1}.Location;
new_pc_loc = (best_est_Rotation*new_xyz'+best_est_Translation)' ;
new_pc = pointCloud(new_pc_loc, 'Color', office{1}.Color);
subplot(1,2,1), pcshow(office{1}), hold on, pcshow(office{2});
subplot(1,2,2), pcshow(new_pc), hold on, pcshow(office{2});


