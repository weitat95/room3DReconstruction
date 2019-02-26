%% Load the office1.mat data
office = load('data/office1.mat');
office = office.pcl_train;

%% Uncomment to load the test file
% office = load('office2.mat');
% office = office.pcl_test;
close all;
for i = 1:length(office) % Reading the 40 point-clouds
    pc = office{i};
    if i==27
        final_bin_mask = clear_noise(pc, true);
    else
        final_bin_mask = clear_noise(pc, false);
    end
    
    % Plot 2d Original
    ori_color = pc.Color;
    ori_loc = pc.Location;
    figure(1);
    subplot(1,2,1),
    before_rgb = imag2d(pc.Color); 
    imshow(before_rgb);
    
    % Plot Masked 2d
    indx_xyz_no = find(final_bin_mask==0);
    
    color_pc_fin = ori_color;
    xyz_pc_fin = ori_loc;
    color_pc_fin(indx_xyz_no, :) = 0;
    new_pc = pointCloud(xyz_pc_fin, 'Color', color_pc_fin);
    subplot(1,2,2),
    imshow(imag2d(new_pc.Color));
    
    % Plot Original Pc
    figure(2);
    subplot(1,2,1),
    pcshow(pc);
    
    % Plot Removed Pc
    color_pc_fin(indx_xyz_no, :) = [];
    xyz_pc_fin(indx_xyz_no, :) = [];
    new_pc = pointCloud(xyz_pc_fin, 'Color', color_pc_fin);
    subplot(1,2,2),
    pcshow(new_pc);
    
    pause
end