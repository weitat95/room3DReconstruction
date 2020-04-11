function [extreme_index, DoG, L] = extrema_points_new(img,sigma, n , k)
 
%DoG = {};
DoG = [];
L = [];
%DoG{1} = img - imgaussfilt(img,sigma);

DoG(:,:,1) = img - imgaussfilt(img, sigma);
DoG(:,:,2) = imgaussfilt(img, sigma)- imgaussfilt(img, sigma*k);
L(:,:,1) = img;
L(:,:,2) = imgaussfilt(img,sigma);
L(:,:,3) = imgaussfilt(img,sigma*k);
for i = 1:n
   
    gaus_img1 = imgaussfilt(img, i*k*sigma);
    gaus_img2 = imgaussfilt(img, (i+1)*k*sigma);
 
    %DoG{end+1} =  gaus_img1-gaus_img2;
    
    DoG(:,:,i+2) = gaus_img1-gaus_img2;
    L(:,:,i+3) = gaus_img2;
end
%shape = size(DoG_A);
[row, column, layers] = size(DoG);

for layer = 2:layers-1
    
    extreme_index = [];
    layer_above = DoG(:,:,layer+1);
    layer_below = DoG(:,:,layer-1);
    layer_this = DoG(:,:,layer);
    
    for i = 2:row-1
        for j = 2:column-1
            neighbors =  reshape(layer_above(i-1:i+1, j-1:j+1), 1, []);
            neighbors = [ neighbors, reshape(layer_below(i-1:i+1, j-1:j+1), 1, [])];
            neighbors = [ neighbors, layer_this(i,j-1), layer_this(i,j+1), layer_this(i-1,j-1:j+1), layer_this(i+1,j-1:j+1)];
            me = layer_this(i,j);
            
            if ( me>max(neighbors) || me < min(neighbors))
                extreme_index = [ extreme_index; i, j , layer];
                
                % Getting Rotation Invariance
 
            end
        end
    end
    
end
end
 

