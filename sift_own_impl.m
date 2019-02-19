
% img_ori = imread('mountains.jpeg');
% %img = im2double(img);
% img = rgb2gray(img_ori);
office = load('data/office1.mat');
office = office.pcl_train;
img_ori = imag2d(office{1}.Color);
img = rgb2gray(img_ori);
k = 2^(1/3);
counter=1;

sigma = 0.5;
n=10;

[extreme_index, DoG] = extrema_points(img,sigma, n , k);
%[extreme_index, DoG, L] = extrema_points_new(img, sigma, n, k);


close all;
imshow(img_ori), hold on;
plot(extreme_index(:,2),extreme_index(:,1), 'ro', 'MarkerSize', 20);

%% Hessian matrix

tau = 12; %Unstable Point Extrema Pruning Parameter

[d_x, d_y, d_s] = gradient(DoG);
[d_xx, d_xy, d_xs] = gradient(d_x);
[d_yx, d_yy, d_ys] = gradient(d_y);
[d_sx, d_sy, d_ss] = gradient(d_s);

optimal_points = [];
extreme_offsets = [];
for i = 1:length(extreme_index)
    r = extreme_index(i,1);
    c = extreme_index(i,2);
    s = extreme_index(i,3);
    H_3 = [ d_xx(r,c,s), d_yx(r,c,s), d_sx(r,c,s); 
                d_yx(r,c,s), d_yy(r,c,s), d_sy(r,c,s); 
                d_sx(r,c,s), d_sy(r,c,s), d_ss(r,c,s)];
    inv_H3 = inv(H_3);
    offset_x = -inv_H3*[d_x(r,c,s), d_y(r,c,s), d_s(r,c,s)]';
    
    % Low Contrast Extrema Pruning
    % reject if p < 0.03
    p = abs(DoG(r,c,s)+ 0.5*[d_x(r,c,s) ,d_y(r,c,s), d_s(r,c,s)]*offset_x);
    if (p <0.03)
        display('Reject low contrast')
        continue
    end
    
    % Unstable Point Extrema Pruning
    H_2 = [ d_xx(r,c,s), d_xy(r,c,s);
            d_xy(r,c,s), d_yy(r,c,s)];
    
    if ( det(H_2) < 0 || power(trace(H_2),2)/det(H_2)> tau )
    	display('Reject unstable')
        continue
    end
        
    extreme_offsets = [extreme_offsets; offset_x'];
    if (sum(isnan([offset_x(1), offset_x(2), offset_x(3)]))==0) % if offset is not NaN
        optimal_point = offset_x + [r;c;s]; % add offset to interest point
        optimal_points = [optimal_points; optimal_point']; 
    else % if offset is NaN
        optimal_points = [optimal_points; r,c,s ]; % dont add offset
    end
end

figure(2)
imshow(img_ori), hold on;
plot(optimal_points(:,2),optimal_points(:,1), 'ro', 'MarkerSize', 20);

