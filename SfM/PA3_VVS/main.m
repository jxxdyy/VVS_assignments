clear; clc;
close all;
run('vlfeat-0.9.21/toolbox/vl_setup');
addpath('./Givenfunctions');
addpath('./my_functions');
addpath('./Data')

% intrinsic
K = [1698.873755 0.000000     971.7497705
    0.000000    1698.8796645 647.7488275
    0.000000    0.000000     1.000000];

% load image
sfm01 = imread('0000.JPG');
sfm02 = imread('0001.JPG');
% resize image
sfm01 = imresize(sfm01, 1.5);
sfm02 = imresize(sfm02, 1.5);


% convert to gray scale
g_sfm01 = single(rgb2gray(sfm01));
g_sfm02 = single(rgb2gray(sfm02));


% F = [v, u, scale, orientation]'
% D = 128-dimension descriptor
[F1, D1] = vl_sift(g_sfm01);
[F2, D2] = vl_sift(g_sfm02);
[matches] = vl_ubcmatch(D1, D2, 2.5);    % matches : matched pts idx


matched_pts1 = [F1(1,matches(1,:)); F1(2,matches(1,:)); ones(1, size(matches,2))];
matched_pts2 = [F2(1,matches(2,:)); F2(2,matches(2,:)); ones(1, size(matches,2))];
% matched_pts1 = [F1(1,matches(1,:)); F1(2,matches(1,:)); F1(3,matches(1,:))];
% matched_pts2 = [F2(1,matches(2,:)); F2(2,matches(2,:)); F1(3,matches(2,:))];

%% get Essential matrix & inliers_index
[E, inliers] = get_E_matrix(matched_pts1, matched_pts2, 2500, 1);
inliers_pts1 = matched_pts1(:, inliers);
inliers_pts2 = matched_pts2(:, inliers);

%% get R|t
[R, t] = decomp_E_matrix(E, inliers_pts1, inliers_pts2, K);
det(R)
T = [R t];

%% Triangulation
Points3d = my_triangulation(inliers_pts1, inliers_pts2, K, T);
rgb = get_rgb(inliers_pts1, sfm01);

data = [Points3d; rgb];
SavePLY("SfM_without_outliers2.ply", data);


%% display

f = figure;
figure(f)
imshow(sfm01);
title("sfm image01")


f = figure;
figure(f)
imshow(sfm02);
title("sfm image02")


f = figure;
f.Position(3:4) = [2*f.Position(3) f.Position(4)];
figure(f)
subplot(1,2,1); imshow(sfm01); hold on; 
plot(matched_pts1(1,:),matched_pts1(2,:),'b+','MarkerSize',2); 
subplot(1,2,2); imshow(sfm02); hold on; 
plot(matched_pts2(1,:),matched_pts2(2,:),'r+','MarkerSize',2);
sgtitle("Matched pts with outliers")


f = figure;
f.Position(3:4) = [2*f.Position(3) f.Position(4)];
figure(f)
subplot(1,2,1); imshow(sfm01); hold on; 
plot(inliers_pts1(1,:),inliers_pts1(2,:),'b+','MarkerSize',2); 
subplot(1,2,2); imshow(sfm02); hold on; 
plot(inliers_pts2(1,:),inliers_pts2(2,:),'r+','MarkerSize',2);
sgtitle("Matched pts without outliers")