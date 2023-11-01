function [final_E, inliers] = get_E_matrix(pts1,pts2,iteration,th)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

max_cnt = 0;

for i=1:iteration
    % choose 5-points randomly
    rand_idx = randsample(1:size(pts1,2), 5);
    
    % make p1, p2 (3x5)
    p1 = pts1(:, rand_idx);
    p2 = pts2(:, rand_idx);

    Evec = calibrated_fivepoint(p1, p2);

    for j=1:size(Evec,2)
        E = reshape(Evec(:,j),3,3);

        result = abs(diag(pts2'*E*pts1));
        inliers_tmp = find(result < th);
        cnt_inliers = length(inliers_tmp);

        if cnt_inliers > max_cnt
            max_cnt = cnt_inliers;
            final_E = E;
            inliers = inliers_tmp;
        end
    end
end