function [Points3d] = my_triangulation(inlierPts1,inlierPts2,K,T)
%TRIANGULATION Summary of this function goes here
%   Detailed explanation goes here

Points3d = zeros(3, size(inlierPts1,2));

% R|t
T1 = K*[eye(3) zeros(3,1)];
T2 = K*T;

P11 = T1(1,:);  P12 = T1(2,:); P13 = T1(3,:);
P21 = T2(1,:);  P22 = T2(2,:); P23 = T2(3,:);

for i=1:size(inlierPts1,2)
    % inlier point 1
    x1 = inlierPts1(1,i);
    y1 = inlierPts1(2,i);
    % inlier point 2
    x2 = inlierPts2(1,i);
    y2 = inlierPts2(2,i);

    A = [y1*P13 - P12; P11 - x1*P13; y2*P23 - P22; P21 - x2*P23];
    
    [~, ~, V] = svd(A);
    P_tmp = V(1:3, end);
    
    if P_tmp(3) < 0
        P_tmp = -P_tmp;
    end
    Points3d(:,i) = P_tmp;

end


