
%% Fuse all the points into a single 3D coordinate system

fused_transformation = {};
for i = 1: length(Tform_array)
    i
    frame_transformation = 1;
    for j = i:-1:1
        frame_transformation = frame_transformation * Tform_array{j}.T;
    end
    fused_transformation{end+1} = affine3d(frame_transformation);
end

gridStep = 0.01;
[~, pc_office_1] = clear_noise(office{1}, false);
merged_set = {};
final_pc = pc_office_1;
for i = 2:length(fused_transformation)+1 % Reading the 40 point-clouds
    i
    removeBob = false;
    if i == 27
        removeBob = true;
    end
     
    [~, pc_cleared_noise] = clear_noise(office{i}, removeBob); 
    pc_transformed = pctransform(pc_cleared_noise, fused_transformation{i-1});
    pc_merged = pcmerge(pc_office_1, pc_transformed, gridStep);
    merged_set{end+1} = pc_merged;
    final_pc = pcmerge(final_pc, pc_transformed, gridStep);
    %close all; 
    %pcshow(final_pc);
    
end
close all;
pcshow(final_pc);


% new_xyz = office{2}.Location;
% new_pc_loc = (Rotation_array{1}*new_xyz'+Translation_array{1})' ;
% new_pc = pointCloud(new_pc_loc, 'Color', office{2}.Color);
% 
% Tform = affine3d(horzcat(horzcat(Rotation_array{1}', [0;0;0])', [Translation_array{1};1])');
% Tform = affine3d(horzcat(horzcat(Rotation_array{1}, Translation_array{1})',[0;0;0;1]));
% 
% frame_transformation = 1;
% frame_transformation = frame_transformation * Tform_array{1}.T';
% Tform_2 = affine3d(frame_transformation);
% 
% close all;
% subplot(2,2,1);
% pcshow(office{1}), hold on,
% pcshow(office{2})
% subplot(2,2,2);
% pcshow(office{1}), hold on,
% pcshow(new_pc)
% subplot(2,2,3);
% pcshow(office{1}), hold on,
% pcshow(pctransform(office{2}, Tform))
% subplot(2,2,4);
% pcshow(office{1}), hold on,
% pcshow(pctransform(office{2}, Tform_2))
% 
% pcshow(pcmerge(office{1}, pctransform(office{2}, Tform),0.001))
% 


% for i = 1:length(merged_set)
%     i
%     close all;
%     
%     pcshow(merged_set{i});
%     pause;
% end
% 
% [~,pc_1_denoised ]= clear_noise(office{1}, false);
% for i = 1:length(Tform_array)
%     i 
%     [~,pc_denoised ]= clear_noise(office{i+1}, false);
%     frame_transformation = 1;
%     for j = i:-1:1
%         j
%         frame_transformation = frame_transformation * Tform_array{j}.T;
%         pc_t = pctransform(pc_denoised, affine3d(frame_transformation));
%         close all;
%         subplot(1,2,1), pcshow(pc_1_denoised), hold on, pcshow(pc_denoised);
%         subplot(1,2,2), pcshow(pc_1_denoised), hold on, pcshow(pc_t);
%         pause;
%     end
% end
