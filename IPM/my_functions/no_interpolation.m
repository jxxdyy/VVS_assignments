function [result] = no_interpolation(px_mat,image_val)
%NO_INTERPOLATION Summary of this function goes here
%   Detailed explanation goes here

result = zeros(3, size(px_mat,2));

u = round(px_mat(1,:));
v = round(px_mat(2,:));

for i=1:size(px_mat,2)
    if (0<u(i)) && (u(i)<=1288) && (0<v(i)) && (v(i)<=964)
        result(:,i) = image_val(v(i), u(i),:);
    end
end

