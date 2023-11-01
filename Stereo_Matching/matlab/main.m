clear all; close all;

%% Image Read
% gray
% l_img = imread('../KITTI-Dataset/2011_09_26_drive_0048/unsync_unrect/image_00/data/0000000000.png');
% r_img = imread('../KITTI-Dataset/2011_09_26_drive_0048/unsync_unrect/image_01/data/0000000000.png');

% color
test_l_img = imread('../KITTI-Dataset/2011_09_26_drive_0048/unsync_unrect/image_02/data/0000000000.png');
test_r_img = imread('../KITTI-Dataset/2011_09_26_drive_0048/unsync_unrect/image_03/data/0000000000.png');

% l_img = imread('../KITTI-Dataset/2011_09_26_drive_0048/unsync_unrect/image_02/data/0000000000.png');
% r_img = imread('../KITTI-Dataset/2011_09_26_drive_0048/unsync_unrect/image_03/data/0000000000.png');

% calib
l_img = imread('../KITTI-Dataset/2011_09_26_drive_0048/calibration/image_02/data/0000000000.png');
r_img = imread('../KITTI-Dataset/2011_09_26_drive_0048/calibration/image_03/data/0000000000.png');


[l_img_row,l_img_col, l_ch] = size(l_img);
[r_img_row,r_img_col, r_ch] = size(r_img);


%% Intrinsic
% left gray intrinsic
cam00_K = [9.842439e+02 0.000000e+00 6.900000e+02;
           0.000000e+00 9.808141e+02 2.331966e+02;
           0.000000e+00 0.000000e+00 1.000000e+00];

% right gray intrinsic
cam01_K = [9.895267e+02 0.000000e+00 7.020000e+02;
           0.000000e+00 9.878386e+02 2.455590e+02;
           0.000000e+00 0.000000e+00 1.000000e+00];

% left color intrinsic
cam02_K = [9.597910e+02 0.000000e+00 6.960217e+02;
           0.000000e+00 9.569251e+02 2.241806e+02;
           0.000000e+00 0.000000e+00 1.000000e+00];

% right color intrinsic
cam03_K = [9.037596e+02 0.000000e+00 6.957519e+02;
           0.000000e+00 9.019653e+02 2.242509e+02;
           0.000000e+00 0.000000e+00 1.000000e+00];



%% Distortion [k1, k2, p1, p2, k3]
cam00_dis = [-3.728755e-01 2.037299e-01 2.219027e-03 1.383707e-03 -7.233722e-02];   % left gray dist
cam01_dis = [-3.644661e-01 1.790019e-01 1.148107e-03 -6.298563e-04 -5.314062e-02];  % right gray dist
cam02_dis = [-3.691481e-01 1.968681e-01 1.353473e-03 5.677587e-04 -6.770705e-02];   % left color dist
cam03_dis = [-3.639558e-01 1.788651e-01 6.029694e-04 -3.922424e-04 -5.382460e-02];  % right color dist



%% Camera Param
gl_camParam = cameraParameters("K",cam00_K, "RadialDistortion",cam00_dis(1:2));
gr_camParam = cameraParameters("K",cam01_K, "RadialDistortion",cam01_dis(1:2));
cl_camParam = cameraParameters("K",cam02_K, "RadialDistortion",cam02_dis(1:2));
cr_camParam = cameraParameters("K",cam03_K, "RadialDistortion",cam03_dis(1:2));



%% Undistortion Image
% Bulit-in undistortion
% [ud_l_img, ld] = undistortImage(l_img, cl_camParam);
% [ud_r_img, rd] = undistortImage(r_img, cr_camParam);


% implement
ud_l_img = undistort_image(l_img, l_img_row, l_img_col, cam02_K, cam02_dis);
ud_r_img = undistort_image(r_img, r_img_row, r_img_col, cam03_K, cam03_dis);

ud_cl_img = undistort_image(test_l_img, l_img_row, l_img_col, cam02_K, cam02_dis);
ud_cr_img = undistort_image(test_r_img, r_img_row, r_img_col, cam03_K, cam03_dis);

%% Correspondence Points
[l_matchedPts, r_matchedPts] = get_correspondence_points(ud_l_img, ud_r_img, "ORB");

% get inliers
[F, inliers] = estimateFundamentalMatrix(l_matchedPts, r_matchedPts, NumTrial=2000);

% inlier points
l_inlierPts = l_matchedPts(inliers, :);
r_inlierPts = r_matchedPts(inliers, :);


%% get fundamental matrix
%build-in
%F_amt = F;

% implement
F_mat = get_F_matrix(l_inlierPts, r_inlierPts);


%% get essential matrix
% build-in
%[E, ~] = estimateEssentialMatrix(l_matchedPts, r_matchedPts, cl_camParam, cr_camParam);

% implement
E_mat = get_E_matrix(F_mat, cam02_K, cam03_K);


%% Decompose E into R, t_skew
[t, R] = decomp_E_matrix(E_mat, l_inlierPts, r_inlierPts);


%% Find Rrect1, Rrect2
% built-in
%[l_tform, r_tform] = estimateStereoRectification(F_mat, l_inlierPts.Location, r_inlierPts.Location, size(ud_cr_img));

% implement
[R1, R2] = estimate_Rrect(t, R);


%% Rectify image
% Bulit-in rectification
%[l_rectImg, r_rectImg] = rectifyStereoImages(ud_cl_img, ud_cr_img, l_tform, r_tform);

% Implement
% get Rrect as a calib image and apply it to a test image
[l_rectImg, r_rectImg] = rectify_image(ud_cl_img, ud_cr_img, R1, R2, cam02_K, cam03_K);


% [lR_matchedPts, rR_matchedPts] = get_correspondence_points(ud_cl_img, ud_cr_img, "ORB");
% [F_rect, Rinliers] = estimateFundamentalMatrix(lR_matchedPts, rR_matchedPts, NumTrial=2000);
% lR_inlierPts = lR_matchedPts(Rinliers, :);
% rR_inlierPts = rR_matchedPts(Rinliers, :);


%% Disparity map
disparityRange = [0 48];
disparityMap = get_disparity_map(l_rectImg, r_rectImg, disparityRange);


%% Display 
%original image & distorted image
f1 = figure;
figure(f1); 
montage({l_img, r_img, ud_l_img, ud_r_img})
title("Top : left & right original image / Bottom : left & right undistortion image")


% feature matching with outlier
f2 = figure;
figure(f2)
showMatchedFeatures(ud_l_img,ud_r_img,l_matchedPts,r_matchedPts);
title("Feature matching with outlier")


% feature matchiing without outlier
f3 = figure;
figure(f3)
showMatchedFeatures(ud_l_img, ud_r_img,l_inlierPts, r_inlierPts);
title("Feature matching without outlier")


% left undistorted image with feature points and epilines
f4 = figure;
figure(f4);
imshow(ud_l_img); hold on;
plot(l_inlierPts.Location(:,1), l_inlierPts.Location(:,2), 'go')  % feature points
l_epiLines = epipolarLine(F_mat', r_matchedPts(inliers,:));
l_points = lineToBorderPoints(l_epiLines,size(ud_l_img));
line(l_points(:,[1,3])',l_points(:,[2,4])');
title("Left image epilines")


% right undistorted image with feature points and epilines
f5 = figure;
figure(f5);
imshow(ud_r_img); hold on;
plot(r_inlierPts.Location(:,1), r_inlierPts.Location(:,2), 'go')  % feature points
r_epiLines = epipolarLine(F_mat, l_matchedPts(inliers,:));
r_points = lineToBorderPoints(r_epiLines,size(ud_r_img));
line(r_points(:,[1,3])',r_points(:,[2,4])');
title("Right image epilines")


% left rect image with feature points and epilines
f6 = figure;
figure(f6);
imshow(l_rectImg); %hold on;
% plot(lR_inlierPts.Location(:,1), lR_inlierPts.Location(:,2), 'go')  % feature points
% lR_epiLines = epipolarLine(F_rect', rR_matchedPts(Rinliers,:));
% lR_points = lineToBorderPoints(lR_epiLines,size(l_rectImg));
% line(lR_points(:,[1,3])',lR_points(:,[2,4])');
title("Left rect image")

% right rect image with feature points and epilines
f7 = figure;
figure(f7);
imshow(r_rectImg); %hold on;
% plot(rR_inlierPts.Location(:,1), rR_inlierPts.Location(:,2), 'go')  % feature points
% rR_epiLines = epipolarLine(F_rect, lR_matchedPts(Rinliers,:));
% rR_points = lineToBorderPoints(rR_epiLines,size(r_rectImg));
% line(rR_points(:,[1,3])',rR_points(:,[2,4])');
title("Right rect image")


% disparity map
f8 = figure;
figure(f8)
imshow(disparityMap, disparityRange)
title("Disparity Map")
colormap jet
colorbar