function [final_R, final_t] = decomp_E_matrix(E_Matrix, inlierPts1, inlierPts2, K)
%DECOMP_E_MATRIX Summary of this function goes here
%   Detailed explanation goes here

% SVD decomposition
[U,S,VT] = svd(E_Matrix);

W = [0 -1 0;
     1 0 0;
     0 0 1];
 
R1 = U*W*VT; 
R2 = U*W'*VT;
t1 = U(:,3);
t2 = -U(:,3);

% tx1 = U*W*S*U';
% tx2 = U*W'*S*U';
% t1 = [tx1(3,2) tx1(1,3) tx1(2,1)]'
% t2 = [tx2(3,2) tx2(1,3) tx2(2,1)]'

% d1 = det(R1); 
% d2 = det(R2);

% normarlized image plane
n1 = inv(K)*inlierPts1;
n2 = inv(K)*inlierPts2;
n1 = [n1; ones(1, size(n1, 2))];
n2 = [n2; ones(1, size(n2, 2))];
nn = [n1 n2];

% camera matrix
cm1 = [R1 t1]; 
cm2 = [R1 t2]; 
cm3 = [R2 t1]; 
cm4 = [R2 t2];

% decision
test1 = cm1*nn;
test2 = cm2*nn;
test3 = cm3*nn;
test4 = cm4*nn;

% validation
if sum(test1 < 0, 'all') == 0
    final_R = R1; 
    final_t = t1;
elseif sum(test2 < 0, 'all') == 0
    final_R = R1;
    final_t = t1;
elseif sum(test3 < 0, 'all') == 0
    final_R = R2;
    final_t = t1;
elseif sum(test4 < 0, 'all') == 0
    final_R = R2;
    final_t = t2;
else
    disp("=================Invalid camera parameters, please run again!!=================")
end




