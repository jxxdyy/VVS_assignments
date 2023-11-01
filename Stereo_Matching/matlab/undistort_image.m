function [ud_img] = undistort_image(d_img, row, col, K, D)
%UNDISTORTION Summary of this function goes here

%   Detailed explanation goes here
ud_img = zeros(row, col, 3); % undistortion img

% img pixel mat
[v_arr, u_arr] = meshgrid(0:row-1, 0:col-1);
u_arr = reshape(u_arr, [1, row*col]);
v_arr = reshape(v_arr, [1, row*col]);

ud_px_mat = [u_arr; v_arr; ones(1, row*col)]; % undist img의 pixel matrix (3*n)
ud_n_mat = inv(K)*ud_px_mat;                  % undist img의 normalized matrix (3*n)


r_u2 = ud_n_mat(1,:).*ud_n_mat(1,:) + ud_n_mat(2,:).*ud_n_mat(2,:);
r_u4 = r_u2.*r_u2;
r_u6 = r_u2.*r_u4;

k1 = D(1); k2 = D(2); p1 = D(3); p2 = D(4); k3 = D(5);

% radial & tangential 
r_d = 1 + k1*r_u2 + k2*r_u4 + k3*r_u6;
t_d = [2*p1*ud_n_mat(1,:).*ud_n_mat(2,:) + p2*(r_u2 + 2*ud_n_mat(1,:).*ud_n_mat(1,:));
       p1*(r_u2 + 2*ud_n_mat(2,:).*ud_n_mat(2,:)) + 2*p2*ud_n_mat(1,:).*ud_n_mat(2,:)];


d_n_mat = r_d.*ud_n_mat(1:2, :) + t_d;                  % dist img의 normalized matrix (3*n)
d_px_mat = round((K*[d_n_mat; ones(1, row*col)]))';     % dist img의 pixel matrix (3*n)

% enter the color value
i = 1;
for v=1:row
    for u=1:col
        if (0 < d_px_mat(i,1)) && (d_px_mat(i,1) <= col) && (0 < d_px_mat(i,2)) && (d_px_mat(i,2) <= row)
            ud_img(v, u, 1) = d_img(d_px_mat(i,2),d_px_mat(i,1), 1);
            ud_img(v, u, 2) = d_img(d_px_mat(i,2),d_px_mat(i,1), 2);
            ud_img(v, u, 3) = d_img(d_px_mat(i,2),d_px_mat(i,1), 3);
        end
        i = i+1;
    end
end

ud_img = uint8(ud_img);
