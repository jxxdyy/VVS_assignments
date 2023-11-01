clear;
clc;
close all;
addpath('./pfunctions');
addpath('./my_functions');
load('./data/calib_result.mat');


%%%%%%%%%%%%%%%%%%% You can set the parameters here.%%%%%%%%%%%%%%%%%%% 
%% Define parameters
s = 4;  % scale
a = 2;  % amp
blending_range1 = [50 25 25 25];   % blending range of mask1
blending_range2 = [15 25 25 25];   % blending range of mask1



%% Load Checkerboard
im = imread('./data/plane_pattern.jpg');  % 1288*964*3 image

% Undistort image
disp('Undistort image');

roi = [740,906,558,814];
chkim = zeros(size(im,1), size(im,2), 1);
chkim(roi(1):roi(2),roi(3):roi(4)) = 1;
rim = im(:,:,1); gim = im(:,:,2); bim = im(:,:,3);
rim(chkim == 0) = 0;
gim(chkim == 0) = 0;
bim(chkim == 0) = 0;
tim(:,:,1) = rim; tim(:,:,2) = gim; tim(:,:,3) = bim;

%% Detect Checkerboard Points 
disp('Detect checkerboard points');
imagePoints = detectCheckerboardPoints(tim);
f = figure;
figure(f);
imshow(im, 'InitialMagnification', 50); hold on;
plot(imagePoints(:, 1), imagePoints(:, 2), '*-g');
hold off;
title("Detect checkerboard points")

%% Establish world X and image pixel x 
disp('Optimize camera0 pose');
%yinc = repmat([50,0], 8,1);

% Q1. Insert Your Code Here! %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO:  Establish world X and image pixel x
% INPUT: Your define coordinate
% OUTPUT: Homogeneous world coordinate (variable: world)
% [10 Point]


[x, y, z] = meshgrid(linspace(0, 350, 8), linspace(0, 350, 8), 1);
groundpoints = [x(:) y(:)];
world = [x(:) y(:) z(:)]';

f = figure;
figure(f);
scatter3(x,y,z,3, "black")
title("Homogeneous world coordinate")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Compute Relative Pose (Rt_p0) btw, Cam0 and World origin
lb = [-pi,-pi,-pi,-1000,-1000,-1000];
ub = [pi,pi,pi,1000,1000,1000];
x0 = [-pi/2,0,0,0,0,1000];
options=optimset('LargeScale','on','Display','iter','TolFun',1e-10,'TolX',1e-10,'MaxFunEvals',100000);
[x,resnorm,residual,exitflag,output] = lsqnonlin(@(x)RTfun(x, world, imagePoints(:,1:2)', IntParam0), ...
                                                 x0, lb, ub, options);


%% Evaluation Rt_p0 
Rt_p1 = SetAxis(x);
Rt_p0 = [0.9986 -0.0519 0.0026 -142.2512;
         -0.0038 -0.0227 0.9997 895.9244;
         -0.0518 -0.9984 -0.0228 917.7570;
         0 0 0 1];

% R = Rt_p1(1:3, 1:3); t = Rt_p1(1:3, end);
% test = eye(4);
% test(1:3, 1:3) = R';
% test(1:3, end) = -R'*t;


% RT * groundpoints (gound : z=0)
temp = Rt_p0*[world(1:2,:);zeros(1,size(world,2));ones(1,size(world,2))];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q2. Implement Your Code Here ! 
% TODO: Add distortion to world points.
% [10 Point]
% INPUT and OUTPUT : see the following AddDistortion_fisheye
% Distortion model Please see. https://euratom-software.github.io/calcam/html/intro_theory.html#fisheye-lens-distirtion-model

% Please make your own AddDistortion_fisheye function
%[xx,yy] = AddDistortion_fisheye(temp(1,:)./temp(3,:),temp(2,:)./temp(3,:),IntParam0(6:end),temp(3,:)<0);

[xx,yy] = my_distortion_fisheye(temp, IntParam0);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%IntParam = [fx, skew, cx, fy, cy, k1, k2, k3, k4]%
u=IntParam0(1)*xx+IntParam0(2)*yy+IntParam0(3);
v=IntParam0(4)*yy+IntParam0(5);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q3. Implement Your Code Here ! 
% TODO : Display Reprojection Error with Pixel Error Metric
% INPUT : reprojected point (variable: u,v), and detected point (variable: imagePoints)
% [10 Point]

% Plot imagePoints and u,v for comparison
f = figure;
figure(f);
plot(u,-v, 'ok'); hold on;
plot(imagePoints(:, 1), -imagePoints(:, 2), '*r');
hold off;
title("Reprojection error with pixel error metric")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Prepare pixel points for AVM image

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q4. Implement Your Code Here ! 
% TODO : Create Your Own Image Coordinate to make a IPM image
% INPUT : Your Define Image Coordinate
% OUTPUT : Pixel Position (wx, wy) 
% [10 Point]

% [Input]
% Define your own image size for top-view (ex, 2,000*2,000)
% [Output]
% wx, 1x(2,000*2,000) 1x(# of pixels)
% wy, 1x(2,000*2,000) 1x(# of pixels)

scale = s;
amp = a;
shift_x = 100*amp;
shift_y = 100*amp;
[wx, wy] = meshgrid(-999-shift_x:scale:1350+shift_x, -199-shift_y:scale:2500+shift_y);
cols = size(wx,2);
rows = size(wy,1);
wx = wx(:)';
wy = wy(:)';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Linear blend boundary
im0 = imread('./data/cam_f.jpg');
im1 = imread('./data/cam_r.jpg');
im2 = imread('./data/cam_b.jpg');
im3 = imread('./data/cam_l.jpg'); 
f = figure;
figure(f);
subplot(2,2,1); imshow(im0);
subplot(2,2,2); imshow(im1);
subplot(2,2,3); imshow(im2);
subplot(2,2,4); imshow(im3);


%% Top_view from camera front

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q5. Implement Your Code Here ! 
% TODO : Compute Relative Pose btw. world and front camera
% INPUT : wx, wy
% OUTPUT : 2D position(variable: temp) in camera coordinate corresponding to (wx, wy) 
% [2.5 Point]

% HINT: Make wx, wy to homogeneous coordindate system
% [Input]
% wx, 1x(2,000*2,000) 1x(# of pixels)
% wy, 1x(2,000*2,000) 1x(# of pixels)
% [Output]
% temp, 4x(2,000*2,000) 4x(# of pixels)

temp = Rt_p0*[wx; wy; zeros(1, length(wx)); ones(1, length(wx))];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Use the result of Q2.
%[wxx,wyy]=AddDistortion_fisheye(temp(1,:)./temp(3,:),temp(2,:)./temp(3,:),IntParam0(6:end),temp(3,:)<0);

% Use my distortion
[wxx,wyy] = my_distortion_fisheye(temp, IntParam0);
u0=IntParam0(1)*wxx+IntParam0(2)*wyy+IntParam0(3) + 1;
v0=IntParam0(4)*wyy+IntParam0(5) + 1;



% undi_image = undist_image(u0, v0, im0);
% f = figure;
% figure(f)
% imshow(undi_image)
% title("undistortion image")


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q6. Implement Your Code Here ! 
% TODO: Image Generation by Backward warping.
% INPUT and OUTPUT : Please see the following Interpolation4_Color() function
% [10 Point]

% [Input]
% u0, 1x(2,000*2,000) 1x(# of pixels)
% v0, 1x(2,000*2,000) 1x(# of pixels)
% [Output]
% result0, 3x(2,000*2,000) (color_channel)x(# of pixels)

% Please make your own Interpolation4_Color function



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%result0=Interpolation4_Color([u0;v0],double(im0));
%result0 = no_interpolation([u0; v0], double(im0));
result0 = my_Interpolation4_Color([u0;v0],double(im0));

for n=1:3
    top_view0(:,:,n)=reshape(result0(n,:),rows,cols);
end
top_view0(uint32(rows/2)-uint32(150/scale):end,:,:) = 0;
top_view0 = uint8(top_view0);

f = figure;
figure(f);
imshow(top_view0); drawnow;
title('Top view from front camera')


%% Top_view from camera right

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q7. Implement Your Code Here ! 
% TODO : Compute Relative Pose btw. world and right camera
% INPUT : wx, wy
% OUTPUT : 2D position(variable: temp) in camera coordinate corresponding to (wx, wy) 
% [2.5 Point]

temp = CameraRelative01 * Rt_p0 * [wx; wy; zeros(1, length(wx)); ones(1, length(wx))];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Use the result of Q2.
%[wxx,wyy]=AddDistortion_fisheye(temp(1,:)./temp(3,:),temp(2,:)./temp(3,:),IntParam1(6:9),temp(3,:)<0);

% Use my distortion
[wxx,wyy] = my_distortion_fisheye(temp, IntParam1);
u1=IntParam1(1)*wxx+IntParam1(2)*wyy+IntParam1(3) + 1;
v1=IntParam1(4)*wyy+IntParam1(5) + 1;


% Use the result of Q6
%result1=Interpolation4_Color([u1;v1],double(im1));
%result1 = no_interpolation([u1; v1], double(im1));
result1 = my_Interpolation4_Color([u1;v1],double(im1));

for n=1:3
    top_view1(:,:,n)=reshape(result1(n,:),rows,cols);
end
top_view1(:,1:uint32(cols/2)+uint32(100/scale),:) = 0;
top_view1 = uint8(top_view1);

f = figure;
figure(f); 
imshow(top_view1);
title('Top view from right camera')


%% Top_view from camera back

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q8. Implement Your Code Here ! 
% TODO : Compute Relative Pose btw. world and back camera
% INPUT : wx, wy
% OUTPUT : 2D position(variable: temp) in camera coordinate corresponding to (wx, wy) 
% [2.5 Point]

temp = CameraRelative12 * CameraRelative01 * Rt_p0 * [wx; wy; zeros(1, length(wx)); ones(1, length(wx))];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Use the result of Q2.
% [wxx,wyy]=AddDistortion_fisheye(temp(1,:)./temp(3,:),temp(2,:)./temp(3,:),IntParam2(6:9),temp(3,:)<0);

% Use my distortion
[wxx,wyy] = my_distortion_fisheye(temp, IntParam2);
u2=IntParam2(1)*wxx+IntParam2(2)*wyy+IntParam2(3) + 1;
v2=IntParam2(4)*wyy+IntParam2(5) + 1;


% Use the result of Q6
%result1=Interpolation4_Color([u2;v2],double(im2));
%result2 = no_interpolation([u2; v2], double(im2));
result2 = my_Interpolation4_Color([u2;v2],double(im2));

for n=1:3
    top_view2(:,:,n)=reshape(result2(n,:),rows,cols);
end
top_view2(1:uint32(rows/2)+uint32(100/scale),:,:) = 0;
top_view2 = uint8(top_view2);

f = figure;
figure(f); 
imshow(top_view2);
title('Top view from rear camera')


%% Top_view from camera left

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q9. Implement Your Code Here ! 
% TODO : Compute Relative Pose btw. world and left camera
% INPUT : wx, wy
% OUTPUT : 2D position(variable: temp) in camera coordinate corresponding to (wx, wy) 
% [2.5 Point]

temp = inv(CameraRelative30) * Rt_p0 * [wx; wy; zeros(1, length(wx)); ones(1, length(wx))];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Use the result of Q2.
% [wxx,wyy]=AddDistortion_fisheye(temp(1,:)./temp(3,:),temp(2,:)./temp(3,:),IntParam3(6:9),temp(3,:)<0);

% Use my distortion
[wxx,wyy] = my_distortion_fisheye(temp, IntParam3);
u3=IntParam3(1)*wxx+IntParam3(2)*wyy+IntParam3(3) + 1;
v3=IntParam3(4)*wyy+IntParam3(5) + 1;


% Use the result of Q6.
%result3=Interpolation4_Color([u3;v3],double(im3));
%result3 = no_interpolation([u3; v3], double(im3));
result3 = my_Interpolation4_Color([u3;v3],double(im3));

for n=1:3
    top_view3(:,:,n)=reshape(result3(n,:),rows,cols);
end
top_view3(:,uint32(cols/2)-uint32(100/scale):end,:) = 0;
top_view3 = uint8(top_view3);

f = figure;
figure(f);
imshow(top_view3);
title('Top view from left camera')




%% All around view

% elements of surround view
surround_view_sub1 = top_view0 + top_view2; % top & bottom
surround_view_sub2 = top_view1 + top_view3; % left & right

% blending masks
blend_mask1 = ones(rows, cols); % apply to sround view sub1
blend_mask2 = ones(rows, cols); % apply to sround view sub2

% sub_mask parameters
if rem(rows, 2) == 0
    mask_r = rows/2;
else
    mask_r = fix(rows/2)+1;
end

if rem(cols, 2) == 0
    mask_c = cols/2;
else
    mask_c = fix(cols/2)+1;
end

ran1 = blending_range1;   % blending range of mask1 : [top_left, top_right, bottom_left, bottom_right]
ran2 = blending_range2;   % blending range of mask2 : [left_top, right_top, left_bottom, right_bottom]

ran1_tl = ran1(1); ran1_tr = ran1(2); ran1_bl = ran1(3); ran1_br = ran1(4);
ran2_lt = ran2(1); ran2_rt = ran2(2); ran2_lb = ran2(3); ran2_rb = ran2(4);

% sub_mask of blend_mask1
mask1_tl = get_blending_mask(mask_r, mask_c, ran1_tl, ran2_lt);
mask1_tr = flip(get_blending_mask(mask_r, mask_c, ran1_tr, ran2_rt), 2);
mask1_bl = flip(get_blending_mask(mask_r, mask_c, ran1_bl, ran2_lb), 1);
mask1_br = flip(flip(get_blending_mask(mask_r, mask_c, ran1_br, ran2_rb), 1), 2);

% make blend_mask1
blend_mask1(1:mask_r, 1:mask_c) = mask1_tl;
blend_mask1(1:mask_r, cols-mask_c+1:end) = mask1_tr;
blend_mask1(rows-mask_r+1:end, 1:mask_c) = mask1_bl;
blend_mask1(rows-mask_r+1:end, cols-mask_c+1:end) = mask1_br;


% sub_mask of blend_mask2
mask2_lt = flip(flip(get_blending_mask(mask_r, mask_c, ran2_lt, ran1_tl), 1), 2);
mask2_rt = flip(get_blending_mask(mask_r, mask_c, ran2_rt, ran1_tr), 1);
mask2_lb = flip(get_blending_mask(mask_r, mask_c, ran2_lb, ran1_bl), 2);
mask2_rb = get_blending_mask(mask_r, mask_c, ran2_rb, ran1_br);

% make blend_mask2
blend_mask2(1:mask_r, 1:mask_c) = mask2_lt;
blend_mask2(1:mask_r, cols-mask_c+1:end) = mask2_rt;
blend_mask2(rows-mask_r+1:end, 1:mask_c) = mask2_lb;
blend_mask2(rows-mask_r+1:end, cols-mask_c+1:end) = mask2_rb;

% get surround_view_sub1
surround_view_sub1 = double(surround_view_sub1).*blend_mask1;
surround_view_sub1 = uint8(surround_view_sub1);

% get surround_view_sub2
surround_view_sub2 = double(surround_view_sub2).*blend_mask2;
surround_view_sub2 = uint8(surround_view_sub2);

% get overall surround view
surround_view = surround_view_sub1 + surround_view_sub2;


% visualizing
f = figure;
figure(f)
imshow(blend_mask1)
title("blending mask 1")

f = figure;
figure(f)
imshow(blend_mask2)
title("blending mask 2")

f = figure;
figure(f)
imshow(surround_view_sub1)
title("Surround view sub1")

f = figure;
figure(f)
imshow(surround_view_sub2)
title("Surround view sub2")

f = figure;
figure(f)
imshow(surround_view)
title("Surround view")

