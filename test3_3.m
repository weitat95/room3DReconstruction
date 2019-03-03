
%% Fuse all the points into a single 3D coordinate system

fused_transformation = {};

for i = 1: length(Translation_array)
    i
    frame_transformation = ones('like', ICP_array{i}.T);
    for j = 1:i
        frame_transformation = frame_transformation * ICP_array{j}.T;
    end
    fused_transformation{end+1} = affine3d(frame_transformation);
end

gridStep = 1.0;
[~, final_pc] = clear_noise(office{1}, false);

for i = 2:length(office)-1 % Reading the 40 point-clouds
    i
    removeBob = false;
    if i == 26
        removeBob = true;
    end
    
    [~, pc_cleared_noise] = clear_noise(office{i}, removeBob); 
    pc_transformed = pctransform(pc_cleared_noise, fused_transformation{i-1});
    final_pc = pcmerge(final_pc, pc_transformed, gridStep);
    
end
close all;
pcshow(final_pc);