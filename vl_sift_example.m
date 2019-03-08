
office = load('data/office1.mat');
office = office.pcl_train;
img_ori = imag2d(office{1}.Color);

% img_ori = imread('mountains.jpeg');

img = rgb2gray(img_ori) ;
img = single(img) ;

[f,d] =vl_sift(img) ;

figure(1)
image(img_ori);
h1 = vl_plotframe(f) ;
set(h1,'color','k','linewidth',3) ;
 
figure(2)
image(img_ori);
h2 = vl_plotframe(f) ;
set(h2,'color','y','linewidth',2) ;

figure(3)
image(img_ori);
h3 = vl_plotsiftdescriptor(d,f);
set(h3, 'color', 'g');