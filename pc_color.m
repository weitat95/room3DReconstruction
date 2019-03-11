function pc_out = pc_color(pc,rgb)
%PC_COLOR Summary of this function goes here
%   Detailed explanation goes here
final_xyz = pc.Location;
final_color = pc.Color;

final_color = repmat(rgb,length(final_color),1);
pc_out = pointCloud(final_xyz, 'Color', final_color);
end

