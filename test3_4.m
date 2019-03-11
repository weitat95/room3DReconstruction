%% Using pcfitplane
MAXANGULARDIST = 5;
MAXDISTANCE = 0.5;
PLANE_M_NORMAL = [0.9099, 0.1237, 0.3960];
PLANE_R_NORMAL = [0.4990, 0.1896, -0.8456];
PLANE_L_NORMAL = [-0.5939, 0.0031, 0.8046];
PLANE_CEI_NORMAL = [-0.0379, -0.9581, -0.2839];

CENTROID_R = [1.27401, -0.312675, -0.565];
CENTROID_L = [0.084, -0.3772, 2.458];
%referenceVector = [0,0,1];


[model1,inlierIndices,outlierIndices] = pcfitplane(final_pc, MAXDISTANCE, PLANE_M_NORMAL);
plane1 = select(final_pc,inlierIndices);
remainPtCloud = select(final_pc,outlierIndices);

[model2,inlierIndices2,outlierIndeces2] = pcfitplane(remainPtCloud, MAXDISTANCE, PLANE_R_NORMAL);
plane2 = select(remainPtCloud,inlierIndices2);
remainPtCloud2 = select(remainPtCloud,outlierIndeces2);

[model3,inlierIndices3,outlierIndeces3] = pcfitplane(remainPtCloud2, MAXDISTANCE, PLANE_L_NORMAL);
plane3 = select(remainPtCloud2,inlierIndices3);
remainPtCloud3 = select(remainPtCloud2,outlierIndeces3);

%% Find Ceiling
ceiling_roi = [ -inf,inf ; -inf, 0.1; -inf,inf];
ceilingIndices = findPointsInROI(remainPtCloud3, ceiling_roi);

ceiling_plane = select(remainPtCloud3, ceilingIndices);

[ceiling_model, ceilingIndices, ~] = pcfitplane(remainPtCloud3, MAXDISTANCE*5, 'SampleIndices', ceilingIndices);

%% Remove Ceiling

remain_xyz = remainPtCloud3.Location;
remain_color = remainPtCloud3.Color;

remain_xyz(ceilingIndices, :) = [];
remain_color(ceilingIndices, :) = [];

pc_ceiling_removed = pointCloud(remain_xyz, 'Color', remain_color);

subplot(2,2,1), pcshow(plane1);
subplot(2,2,2), pcshow(plane2);
subplot(2,2,3), pcshow(plane3);
subplot(2,2,4), pcshow(pc_color(plane1,[255,0,0])), hold on,... 
pcshow(pc_color(plane2,[0,255,0])), hold on,...
pcshow(pc_color(plane3,[0,0,255])), hold on,...
pcshow(pc_ceiling_removed);

close all;
pcshow(plane1), hold on,... 
pcshow(plane2), hold on,...
pcshow(plane3), hold on,...
pcshow(pc_ceiling_removed);

%% Find Angle Between Plane

angle_right = acos(PLANE_M_NORMAL * PLANE_R_NORMAL')*180/pi;
angle_left = acos(PLANE_M_NORMAL * PLANE_L_NORMAL')*180/pi;


dist_R_L = min(sqrt(sum((plane3.Location-mean(plane2.Location)).^2,2)));

DIST_COM = sqrt(sum((CENTROID_L-CENTROID_R).^2,2));
