function [extrema_index, DoG_A, theta] = extrema_points(img, sigma, n , k)

DoG = {};
DoG_A = [];

for i = 1:n

    gaus_img1 = imgaussfilt(img, i*sigma);

    gaus_img2 = imgaussfilt(img, (i+1)*k*sigma);
    
    DoG{end+1} =  gaus_img1-gaus_img2;
    
    DoG_A(:,:,i) = gaus_img1-gaus_img2;
    
    %L(:,:,i) = gaus_img1;
end

for layer = 2:length(DoG)-1
    [row, column] = size(DoG{layer});
    extreme_index = [];
    layer_above = DoG{layer+1};
    layer_below = DoG{layer-1};
    layer_this = DoG{layer};
    
    
    for i = 2:row-1
        for j = 2:column-1
            neighbors =  reshape(layer_above(i-1:i+1, j-1:j+1), 1, []);
            neighbors = [ neighbors , reshape(layer_below(i-1:i+1, j-1:j+1), 1, [])];
            neighbors = [ neighbors , layer_this(i,j-1), layer_this(i,j+1), layer_this(i-1,j-1:j+1), layer_this(i+1,j-1:j+1)];
            me = layer_this(i,j);

            if ( me>max(neighbors) || me < min(neighbors))
                extreme_index = [ extreme_index; i, j , layer];
                
                % Getting Rotation Invariance
            end
        end
    end
    
    
    
end
extrema_index = extreme_index;
end

