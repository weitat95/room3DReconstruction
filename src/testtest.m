for i = 1:10
 close all
 
 [~, pc_cleared_noise] = clear_noise(office{1}, removeBob); 
 [~, pc_cleared_noise_2] = clear_noise(office{i+1}, removeBob); 
 pc1 = pc_cleared_noise;
 pc2 = pc_cleared_noise_2;
 pc1 = office{1};
 pc2 = office{i+1};
 trans = 1;
 for j = 1:i
    trans = trans * Tform_array{j}.T;
    tform = affine3d(trans);
 end
 subplot(1,2,1), pcshow(pc1), hold on, pcshow(pc2);
 subplot(1,2,2), pcshow(pctransform(pc2,Tform_array{i})), hold on, pcshow(pc1);
 pause
end