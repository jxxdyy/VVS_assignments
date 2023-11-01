function [undi_image] = undist_image(uu,vv,src_img)
%UNDIST_IMAGE Summary of this function goes here
%   Detailed explanation goes here

undi_image = zeros(775, 668, 3);
%undi_image = zeros(964, 1288, 3);
uu = round(uu);
vv = round(vv);

i = 1;
for u=1:668
    for v=1:775
        if (0 < uu(i)) && (uu(i) <= 1288) && (0 < vv(i)) && (vv(i) <= 964)
            undi_image(v, u, 1) = src_img(vv(i), uu(i), 1);
            undi_image(v, u, 2) = src_img(vv(i), uu(i), 2);
            undi_image(v, u, 3) = src_img(vv(i), uu(i), 3);
        end
        i = i+1;
    end
end
undi_image = uint8(undi_image);

end

