%% Load the office1.mat data
office = load('data/office1.mat');
office = office.pcl_train;

%% Uncomment to load the test file
% office = load('office2.mat');
% office = office.pcl_test;
close all;

taskname='figure/NAN';
for i = 1:length(office) % Reading the 40 point-clouds
    pc = office{i};
    i
    if i==27
        final_bin_mask = clear_noise(pc, true);
    else
        final_bin_mask = clear_noise(pc, false);
    end
    
    % Plot 2d Original
    ori_color = pc.Color;
    ori_loc = pc.Location;
    f1 = figure(1);
    %subplot(1,2,1),
    before_rgb = imag2d(pc.Color);
    imshow(before_rgb);
    %savefig(f1, [taskname, '_original_2d']);
    title('Original_2d');
    
    % Plot Masked 2d
    indx_xyz_no = find(final_bin_mask==0);
    color_pc_fin = ori_color;
    xyz_pc_fin = ori_loc;
    color_pc_fin(indx_xyz_no, :) = 0;
    new_pc = pointCloud(xyz_pc_fin, 'Color', color_pc_fin);
    %subplot(1,2,2),
    f2 = figure(2);
    imshow(imag2d(new_pc.Color));
    %savefig(f2,[taskname, '_masked_2d']);
    title('Masked 2d');

    % Plot Original Pc
    f3 = figure(3);
    %subplot(1,2,1),
    pcshow(pc);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    %savefig(f3,[taskname, '_ori_pc']);
    title('Original PC');
    
    % Plot Removed Pc
    color_pc_fin(indx_xyz_no, :) = [];
    xyz_pc_fin(indx_xyz_no, :) = [];
    new_pc = pointCloud(xyz_pc_fin, 'Color', color_pc_fin);
    %subplot(1,2,2),
    f4 = figure(4);
    pcshow(new_pc);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    %savefig(f4,[taskname, '_cleaned_pc']);
    title('Cleaned PC');
    
    pause
end