function [x_1_out,y_1_out, x_2_out, y_2_out] = sift_denoise(bin_mask_1, bin_mask_2, x_1, y_1 ,x_2, y_2)

xy_1 = zeros(length(x_1), 2);
xy_1(:, 1) = x_1;
xy_1(:, 2) = y_1;

xy_2 = zeros(length(x_2), 2);
xy_2(:, 1) = x_2;
xy_2(:, 2) = y_2;


[r1,c1] = find(bin_mask_1'==0);
[r2,c2] = find(bin_mask_2'==0);
counter=0;
for i=1:length(xy_1)

    if(sum(sum(xy_1(i,:) == [c1,r1],2)==2)~=0)
        counter = counter+1;
        xy_1(i,:)=[-1,-1];
        xy_2(i,:)=[-1,-1];
    end

end
for i=1:length(xy_2)
    if(sum(sum(xy_2(i,:) == [c2,r2],2)==2)~=0)
        counter = counter+1;
        xy_2(i,:)=[-1, -1];
        xy_1(i,:)=[-1, -1];
    end
end
xy_1(xy_1(:,1)==-1,:) = [];
xy_2(xy_2(:,1)==-1,:) = [];

x_1_out = xy_1(:,1);
y_1_out = xy_1(:,2);
x_2_out = xy_2(:,1);
y_2_out = xy_2(:,2);

end

