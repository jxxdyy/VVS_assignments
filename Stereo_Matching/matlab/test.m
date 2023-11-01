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



%% Correspondence Points
[l_matchedPts, r_matchedPts] = get_correspondence_points(ud_l_img, ud_r_img, "ORB");

% get inliers
[F, inliers] = estimateFundamentalMatrix(l_matchedPts, r_matchedPts, NumTrial=2000);

% inlier points
l_inlierPts = l_matchedPts(inliers, :);
r_inlierPts = r_matchedPts(inliers, :);



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

