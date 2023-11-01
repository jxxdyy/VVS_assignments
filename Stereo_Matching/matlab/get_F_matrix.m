function [F] = get_F_matrix(l_inlierPts, r_inlierPts)
%GET_F_MATRIX Summary of this function goes here

%   Detailed explanation goes here
l_colx = l_inlierPts.Location(:,1);  % left image inlier x column
l_coly = l_inlierPts.Location(:,2);  % left image inlier y column
r_colx = r_inlierPts.Location(:,1);  % right image inlier x column
r_coly = r_inlierPts.Location(:,2);  % right image inlier y column

A_col1 = l_colx.*r_colx;
A_col2 = l_colx.*r_coly;
A_col3 = l_colx;
A_col4 = l_coly.*r_colx;
A_col5 = l_coly.*r_coly;
A_col6 = l_coly;
A_col7 = r_colx;
A_col8 = r_coly;
A_col9 = ones(size(l_inlierPts));

A = [A_col1 A_col2 A_col3 A_col4 A_col5 A_col6 A_col7 A_col8 A_col9];

[~,~,V] = svd(A);

F = reshape(V(:, end), [3,3])';

[U_,S_,V_] = svd(F);
S_(3,3) = 0;   % make rank(F) = 2

F = U_*S_*V_';

end

