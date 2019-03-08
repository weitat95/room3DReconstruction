run('vlfeat-0.9.19/toolbox/vl_setup.m');

office = load('data/office1.mat');
office = office.pcl_train;


img_ori_1 = imag2d(office{1}.Color);
img_ori_2 = imag2d(office{2}.Color);

%% Extracting frames and descriptors
I = single(rgb2gray(img_ori_1));
[f,d] = vl_sift(I);
I2 = single(rgb2gray(img_ori_2));
[f2,d2] = vl_sift(I2);

% Visualize feature
subplot(1,2,1);
image(img_ori_1);
h1 = vl_plotframe(f);
set(h1, 'color', 'y', 'linewidth', 3);

subplot(1,2,2);
image(img_ori_2);
h2 = vl_plotframe(f2);
set(h2, 'color', 'y', 'linewidth', 3);

% Basic matching
[matches, scores] = vl_ubcmatch(d,d2);

close all;
subplot(1,2,1);
image(img_ori_1);
h1_1 = vl_plotframe(f(:,(matches(1,:))));
set(h1_1, 'color', 'y', 'linewidth', 3);

subplot(1,2,2);
image(img_ori_2);
h2_2 = vl_plotframe(f2(:,(matches(2,:))));
set(h2_2, 'color', 'y', 'linewidth', 3);
close all;

imagesc([img_ori_1, img_ori_2]);

h1_1 = vl_plotframe(f(:,(matches(1,:))));
set(h1_1, 'color', 'y', 'linewidth', 3);

%h2_2 = vl_plotframe(f2(:,(matches(2,:))));
%set(h2_2, 'color', 'y', 'linewidth', 3);

[~, offset,~]= size(img_ori_1);

x1s = int32(f(1,(matches(1,:))));
y1s = int32(f(2,(matches(1,:))));

x2s = int32(f2(1,(matches(2,:))));
y2s = int32(f2(2,(matches(2,:))));
hold on;

plot(x2s+offset,y2s,'ro','MarkerSize',10);

hold on;
perm = randperm(length(x1s));
sel = perm(1:20); % select 20 random points
for i = 1:length(sel)
    x1 = x1s(sel(i));
    y1 = y1s(sel(i));
    x2 = x2s(sel(i))+offset;
    y2 = y2s(sel(i));
    plot([x1,x2],[y1 y2], 'r', 'LineWidth', 2);
end



