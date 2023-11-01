function [l_matchedPts, r_matchedPts] = get_correspondence_points(l_img, r_img, feat_opt)
%GET_CORRESPONDENCE_POINTS Summary of this function goes here

%   Detailed explanation goes here
gray_l_img = rgb2gray(l_img);
gray_r_img = rgb2gray(r_img);

if feat_opt == "ORB"
    l_points = detectORBFeatures(gray_l_img);
    r_points = detectORBFeatures(gray_r_img);
elseif feat_opt == "SIFT"
    l_points = detectSIFTFeatures(gray_l_img);
    r_points = detectSIFTFeatures(gray_r_img);
elseif feat_opt == "BRISK"
    l_points = detectBRISKFeatures(gray_l_img);
    r_points = detectBRISKFeatures(gray_r_img);
elseif feat_opt == "HARRIS"
    l_points = detectHarrisFeatures(gray_l_img);
    r_points = detectHarrisFeatures(gray_r_img);
end

[l_feat, l_valid_pts] = extractFeatures(gray_l_img, l_points);
[r_feat, r_valid_pts] = extractFeatures(gray_r_img, r_points);

indexPairs = matchFeatures(l_feat, r_feat);
l_matchedPts = l_valid_pts(indexPairs(:,1),:);
r_matchedPts = r_valid_pts(indexPairs(:,2),:);




