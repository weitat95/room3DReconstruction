%% Irrevelant Points outside window

office = load('data/office1.mat');
office = office.pcl_train;

%% Uncomment to load the test file
% office = load('office2.mat');
% office = office.pcl_test;
%%
close all;
for i = 20:length(office) % Reading the 40 point-clouds

i=27
rgb = office{i}.Color; % Extracting the colour data
point = office{i}.Location; % Extracting the xyz data
pc = pointCloud(point, 'Color', rgb); % Creating a point-cloud variable

%% Thresholding
loc = pc.Location;
color_pc=pc.Color;
z_loc = loc(:,3);
% indx_xyz_no = find(z_loc>3.5 );
indx_xyz_no = find(z_loc>3.5 | isnan(z_loc));
indx_xyz_yes = find(z_loc<=3.5);

xyz_pc = pc.Location;

%xyz_pc(indx_xyz_no,:) = [];
color_pc(indx_xyz_no,:) = 0;
%color_pc(indx_xyz_yes,:) = 255;

% Creating a new point-cloud
new_pc_1 = pointCloud(xyz_pc, 'Color', color_pc);

%t1_mask = imag2d(new_pc_1.Color);
%t1_mask_bin = t1_mask(:,:,1);
%t1_mask_bin(t1_mask_bin~=0)=255;
bin = color_pc(:,1)~=0;
t1_mask_bin = reshape(bin, [640, 480])';
% NEW_PC_1 , t1_mask_bin 

%% Task 2
if i==27
    close all;
    obs_mask = ones(480,640);
    obs_mask(83:322,1:480,:)=0;
    
    t2_mask_bin = bsxfun(@times, t1_mask_bin, cast(obs_mask, 'like', t1_mask_bin));
    indx_xyz_no = find(t2_mask_bin==0);
    color_pc(indx_xyz_no,:) = [];
    xyz_pc(indx_xyz_no,:) = [];
    new_pc_2 = pointCloud(xyz_pc, 'Color', color_pc);
    
    loc = new_pc_2.Location;
    z_loc = loc(:,3);
    % indx_xyz_no = find(z_loc>3.5 );
    indx_xyz_no = find(z_loc>3.5 | isnan(z_loc));
    indx_xyz_yes = find(z_loc<=3.5);

    loc(indx_xyz_no,:) = [];
    col = new_pc_2.Color;
    col(indx_xyz_no,:) = [];
    
    new_pc_22 = pointCloud(loc, 'Color', col);
    pcshow(new_pc_22);
    
    %t2_mask = imag2d(new_pc.Color);
end

%% Task 3

%bwareaopen(new_pc.Location, 50);


figure(1);
before_rgb = imag2d(pc.Color);
figure(2);
pcshow(pc);
figure(3);
pcshow(new_pc);
figure(4);
imag2d(new_pc.Color);
pause
end