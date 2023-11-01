function [result] = my_Interpolation4_Color(px_mat, image_val)
%MY_INTERPOLATION4_COLOR Summary of this function goes here
%   Detailed explanation goes here

% RGB result
result = zeros(3, size(px_mat, 2));
R_result = result(1,:);
G_result = result(2,:);
B_result = result(3,:);

f_px_mat = floor(px_mat);
u = f_px_mat(1,:); v = f_px_mat(2,:);

dist = px_mat - f_px_mat;
u_dist = dist(1,:);     % u distance
v_dist = dist(2,:);     % v distance

% valid index 
valid_idx = find((0 < u) & (u <= 1287) & (0 < v) & (v <= 963)); 
u = u(valid_idx); u_dist = u_dist(valid_idx);
v = v(valid_idx); v_dist = v_dist(valid_idx);


% Convert to lienar index for matrix calculation
Lidx1 = sub2ind(size(image_val), v, u);
Lidx2 = sub2ind(size(image_val), v+1, u);
Lidx3 = sub2ind(size(image_val), v, u+1);
Lidx4 = sub2ind(size(image_val), v+1, u+1);

R = image_val(:,:,1);
G = image_val(:,:,2);
B = image_val(:,:,3);

t1 = [R(Lidx1); G(Lidx1); B(Lidx1)];
t2 = [R(Lidx2); G(Lidx2); B(Lidx2)];
t3 = [R(Lidx3); G(Lidx3); B(Lidx3)];
t4 = [R(Lidx4); G(Lidx4); B(Lidx4)];

% bilinear interpolation
value = (1-u_dist).*(t1.*(1-v_dist) + t2.*v_dist) + u_dist.*(t3.*(1-v_dist) + t4.*v_dist);

R_result(valid_idx) = value(1,:);
G_result(valid_idx) = value(2,:);
B_result(valid_idx) = value(3,:);

% 3*533200 RGB results
result = [R_result; G_result; B_result];


end

