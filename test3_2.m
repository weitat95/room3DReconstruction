%run('../vlfeat-0.9.21/toolbox/vl_setup.m');
%run('vlfeat-0.9.19/toolbox/vl_setup.m');


%% Ransac

office = load('data/office1.mat');
office = office.pcl_train;

Translation_array= {} ;
Rotation_array= {} ;
ICP_array = {} ;
Tform_array = {} ;
for i = 1:length(office)-1 % Reading the 40 point-clouds
    i
    pc2 = office{i};
    pc1 = office{i+1};
    removeBob = false;
    if i ==27
        removeBob = true;
    end
    [~, pc1_cleared] = clear_noise(pc1, removeBob);
    [~, pc2_cleared] = clear_noise(pc2, removeBob);
    [best_est_Translation,  best_est_Rotation, error] = pose_estimation(pc1, pc2, true, false);
    Translation_array{end+1} = best_est_Translation;
    Rotation_array{end+1} = best_est_Rotation;
    Tform_array{end+1} = affine3d(horzcat(horzcat(best_est_Rotation, best_est_Translation)',[0 ;0 ;0 ;1]));
    error
    
    %% Visualise Best Pose Estimation 
    pc1=pc1_cleared;
    pc2=pc2_cleared;
    
    percentage = 0.5;
    gridStep = 0.1;
    Random_Seed = 5;
    
    new_xyz = pc1.Location;
    new_pc_loc = (best_est_Rotation*new_xyz'+best_est_Translation)' ;
    new_pc = pointCloud(new_pc_loc, 'Color', pc1.Color);
        
    %rng(Random_Seed);
    %pc_downsampled_random = pcdownsample(new_pc, 'Random', percentage);
    %[tform_random, pc_icp_random, rmse] = pcregrigid(pc_downsampled_random, pc2);
    %display('Random');
    %rmse
    
    pc_downsampled_grid = pcdownsample(new_pc, 'GridAverage', gridStep);
    pc2_downsampled_grid = pcdownsample(pc2 , 'GridAverage', gridStep);
    [tform_grid, pc_icp_grid, rmse] = pcregrigid(pc_downsampled_grid, pc2_downsampled_grid);
    ICP_array{end+1} = tform_grid;
    rmse
    
    %close all;
    %subplot(2,2,1), pcshow(pc1), hold on, pcshow(pc2), title('Without any transformation');
    %subplot(2,2,2), pcshow(new_pc), hold on, pcshow(pc2), title('With SIFT Transformation');
    %subplot(2,2,3), pcshow(pc_icp_grid), hold on, pcshow(pc2), title('With SIFT+ICP Transformation');
    %subplot(2,2,4), pcshow(pctransform(new_pc, tform_grid)), hold on, pcshow(pc2), title('Not using the returned PC from ICP');
    %pause;
end
