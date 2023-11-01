function [disparityMap] = get_disparity_map(l_rectImg,r_rectImg, disparityRange)
%GET_DISPARITY_MAP Summary of this function goes here

%   Detailed explanation goes here
gray_l_rectImg = rgb2gray(l_rectImg);
gray_r_rectImg = rgb2gray(r_rectImg);

disparityMap = disparitySGM(gray_l_rectImg, gray_r_rectImg, "DisparityRange", disparityRange, "UniquenessThreshold", 20);

end

