function [final_t, final_R] = decomp_E_matrix(E_Matrix, l_inlierPts, r_inlierPts)
%DECOMP_E_MATRIX Summary of this function goes here
%   Detailed explanation goes here

% SVD decomposition
[U,S,V] = svd(E_Matrix);

W = [0 -1 0;
     1 0 0;
     0 0 1];

tx1 = U*W*S*U'; % t_skew
R1 = U*W'*V';   % Rotation

tx2 = U*W'*S*U'; % t_skew
R2 = U*W*V';   % Rotation

t1 = [tx1(3,2) tx1(1,3) tx1(2,1)]';
t2 = [tx2(3,2) tx2(1,3) tx2(2,1)]';

d1 = det(R1);
d2 = det(R2);

if d1 == -1
    R1 = -R1;
end
if d2 == -1
    R2 = -R2;
end


% triangulation
T1 = [R1 t1]; P11 = T1(1,:);  P12 = T1(2,:); P13 = T1(3,:);
T2 = [R1 t2]; P21 = T2(1,:);  P22 = T2(2,:); P23 = T2(3,:);
T3 = [R2 t1]; P31 = T3(1,:);  P32 = T3(2,:); P33 = T3(3,:);
T4 = [R2 t2]; P41 = T4(1,:);  P42 = T4(2,:); P43 = T4(3,:);

% left inlier point (x, y)
xl = l_inlierPts.Location(1,1);
yl = l_inlierPts.Location(1,2);
% right inlier point (x', y')
xr = r_inlierPts.Location(1,1);
yr = r_inlierPts.Location(1,2);

A1 = [yl*P13 - P12; P11 - xl*P13; yr*P13 - P12; P11 - xr*P13];
A2 = [yl*P23 - P22; P21 - xl*P23; yr*P23 - P22; P21 - xr*P23];
A3 = [yl*P33 - P32; P31 - xl*P33; yr*P33 - P32; P31 - xr*P33];
A4 = [yl*P43 - P42; P41 - xl*P43; yr*P43 - P42; P41 - xr*P43];

[~, ~, V1] = svd(A1);
[~, ~, V2] = svd(A2);
[~, ~, V3] = svd(A3);
[~, ~, V4] = svd(A4);

X1 = V1(:, end);
X2 = V2(:, end);
X3 = V3(:, end);
X4 = V4(:, end);

if X1(3) > 0 && X1(4) > 0
    final_R = R1;
    final_t = t1;

elseif X2(3) > 0 && X2(4) > 0
    final_R = R1;
    final_t = t2;

elseif X3(3) > 0 && X3(4) > 0
    final_R = R2;
    final_t = t1;

elseif X4(3) > 0 && X4(4) > 0
    final_R = R2;
    final_t = t2;

else
    final_R = R1;
    final_t = t1;
end

