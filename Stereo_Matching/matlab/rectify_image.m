function [l_rectImg, r_rectImg] = rectify_image(ud_l_img, ud_r_img, R1, R2, K1, K2)
%UNTITLED Summary of this function goes here

%   Detailed explanation goes here
[row, col, ch] = size(ud_l_img);

l_rectImg = zeros(row, col, 3); 
r_rectImg = zeros(row, col, 3);

% img pixel mat
[v_arr, u_arr] = meshgrid(0:row-1, 0:col-1);
u_arr = reshape(u_arr, [1, row*col]);
v_arr = reshape(v_arr, [1, row*col]);

rect_px_mat = [u_arr; v_arr; ones(1, row*col)];  % rect img의 pixel matrix (3*n)

lRect_n_mat = inv(K1)*rect_px_mat;   % left rect img의 normalized matrix (3*n)              
rRect_n_mat = inv(K2)*rect_px_mat;   % right rect img의 normalized matrix (3*n)


inv_lRect = inv(R1)*lRect_n_mat;         % left rect img에 inv R1 적용
l_ud_n_mat = inv_lRect./inv_lRect(3,:);    % left undist img의 normal matrix (3*n)

inv_rRect = inv(R2)*rRect_n_mat;          % right rect img에 inv R2 적용
r_ud_n_mat = inv_rRect./inv_rRect(3,:);    % right undist img의 normal matrix (3*n)
 
l_ud_px_mat = round((K1*l_ud_n_mat))';    
r_ud_px_mat = round((K2*r_ud_n_mat))';

% enter the color value
i = 1;
for v=1:row
    for u=1:col
        if (0 < l_ud_px_mat(i,1)) && (l_ud_px_mat(i,1) <= col) && (0 < l_ud_px_mat(i,2)) && (l_ud_px_mat(i,2) <= row)
            l_rectImg(v, u, 1) = ud_l_img(l_ud_px_mat(i,2), l_ud_px_mat(i,1), 1);
            l_rectImg(v, u, 2) = ud_l_img(l_ud_px_mat(i,2), l_ud_px_mat(i,1), 2);
            l_rectImg(v, u, 3) = ud_l_img(l_ud_px_mat(i,2), l_ud_px_mat(i,1), 3);
        end
        if (0 < r_ud_px_mat(i,1)) && (r_ud_px_mat(i,1) <= col) && (0 < r_ud_px_mat(i,2)) && (r_ud_px_mat(i,2) <= row)
            r_rectImg(v, u, 1) = ud_r_img(r_ud_px_mat(i,2), r_ud_px_mat(i,1), 1);
            r_rectImg(v, u, 2) = ud_r_img(r_ud_px_mat(i,2), r_ud_px_mat(i,1), 2);
            r_rectImg(v, u, 3) = ud_r_img(r_ud_px_mat(i,2), r_ud_px_mat(i,1), 3);

        end
        i = i+1;
    end
end

l_rectImg = uint8(l_rectImg);
r_rectImg = uint8(r_rectImg);

