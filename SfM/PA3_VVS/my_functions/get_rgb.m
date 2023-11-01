function [rgb] = get_rgb(pts,image)
%GET_RGB Summary of this function goes here
%   Detailed explanation goes here

image_r = image(:,:,1);
image_g = image(:,:,2);
image_b = image(:,:,3);

Lidx = sub2ind(size(image), round(pts(2,:)), round(pts(1,:)));

rgb = [image_r(Lidx); image_g(Lidx); image_b(Lidx)];
rgb = double(rgb)/255;

end

