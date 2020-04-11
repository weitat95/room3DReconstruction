%run('../vlfeat-0.9.21/toolbox/vl_setup.m');
run('vlfeat-0.9.19/toolbox/vl_setup.m');

office = load('data/office1.mat');
office = office.pcl_train;

for j=1:length(office)-1   
j

img_ori_1 = imag2d(office{j}.Color);
img_ori_2 = imag2d(office{j+1}.Color);

removeBob = j==27;
[bin_mask_1, pc1_cleared] = clear_noise(office{j}, removeBob);

removeBob = j+1==27;
[bin_mask_2, pc2_cleared] = clear_noise(office{j+1}, removeBob);

%% Extracting frames and descriptors
I = single(rgb2gray(img_ori_1));
[f,d] = vl_sift(I,'PeakThresh', 0);
I2 = single(rgb2gray(img_ori_2));
[f2,d2] = vl_sift(I2,'PeakThresh', 0);

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
[matches, scores] = vl_ubcmatch(d,d2,2.5);

% close all;
% subplot(1,2,1);
% image(img_ori_1);
% h1_1 = vl_plotframe(f(:,(matches(1,:))));
% set(h1_1, 'color', 'y', 'linewidth', 3);
% 
% subplot(1,2,2);
% image(img_ori_2);
% h2_2 = vl_plotframe(f2(:,(matches(2,:))));
% set(h2_2, 'color', 'y', 'linewidth', 3);

close all;

imagesc([img_ori_1, img_ori_2]);
hold on;
[~, offset,~]= size(img_ori_1);

x1s_all = int32(f(1,:));
y1s_all = int32(f(2,:));

x2s_all = int32(f2(1,:));
y2s_all = int32(f2(2,:));

[x1s_all, y1s_all, x2s_all, y2s_all] = sift_denoise(bin_mask_1, bin_mask_2, x1s_all, y1s_all, x2s_all, y2s_all);

plot(x1s_all, y1s_all, 'ro', 'MarkerSize', 10);
hold on;
plot(x2s_all+offset, y2s_all, 'ro', 'MarkerSize', 10);
hold on;
%h1_1 = vl_plotframe(f(:,(matches(1,:))));
%set(h1_1, 'color', 'y', 'linewidth', 3);

%h2_2 = vl_plotframe(f2(:,(matches(2,:))));
%set(h2_2, 'color', 'y', 'linewidth', 3);

x1s = int32(f(1,(matches(1,:))));
y1s = int32(f(2,(matches(1,:))));

x2s = int32(f2(1,(matches(2,:))));
y2s = int32(f2(2,(matches(2,:))));

[x1s, y1s, x2s, y2s] = sift_denoise(bin_mask_1, bin_mask_2, x1s, y1s, x2s,y2s);

hold on;
plot(x1s,y1s,'go','MarkerSize',10);
plot(x2s+offset,y2s,'go','MarkerSize',10);

hold on;
perm = randperm(length(x1s));
n = min([length(x1s),20]);
sel = perm(1:n); % select 20 random points
for i = 1:length(sel)
    x1 = x1s(sel(i));
    y1 = y1s(sel(i));
    x2 = x2s(sel(i))+offset;
    y2 = y2s(sel(i));
    plot([x1,x2],[y1 y2], 'k', 'LineWidth', 2);
end
pause

end
