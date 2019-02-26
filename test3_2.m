%% Ransac

office = load('data/office1.mat');
office = office.pcl_train;

for i = 26:length(office)-1 % Reading the 40 point-clouds
    pc1 = office{i};
    pc2 = office{i+1};
    [~, pc1_cleared] = clear_noise(pc1, false);
    [~, pc2_cleared] = clear_noise(pc2, false);
    [best_est_Translation,  best_est_Rotation, error] = pose_estimation(pc1, pc2, true, false);
    error
    
    %% Visualise Best Pose Estimation 
    pc1=pc1_cleared;
    pc2=pc2_cleared;
    
    close all;
    new_xyz = pc1.Location;
    new_pc_loc = (best_est_Rotation*new_xyz'+best_est_Translation)' ;
    new_pc = pointCloud(new_pc_loc, 'Color', pc1.Color);
    subplot(1,2,1), pcshow(pc1), hold on, pcshow(pc2);
    subplot(1,2,2), pcshow(new_pc), hold on, pcshow(pc2);

    pause;
end