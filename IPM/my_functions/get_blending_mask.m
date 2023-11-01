function [mask] = get_blending_mask(mask_r, mask_c, ran1, ran2)
%GET_BLENDING_MASK Summary of this function goes here
%   Detailed explanation goes here

vertice_sub = [0 0; mask_c 0; mask_c mask_r];
c = vertice_sub(:,1); r = vertice_sub(:,2);

mask = double(roipoly(mask_r, mask_c, c, r)); % temp of blending mask

for v=1:mask_r
    temp = v*mask_c/mask_r;
    for u=1:mask_c
        d = u-temp;
        if (0 <= d) && (d <= ran1)
            val = d/ran1;
            mask(v, u) = 0.5*(1+val);
        end

        if (-ran2 <= d) && (d <= 0)
            val = abs(d/ran2);
            mask(v, u) = 0.5*(1-val);
        end
    end
end


end

