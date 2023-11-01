function [xx,yy] = my_distortion_fisheye(temp, IntParam)
%MY_DISTORTION_FISHEYE Summary of this function goes here
%   Detailed explanation goes here

% normalized
norm_x = temp(1,:)./temp(3,:);
norm_y = temp(2,:)./temp(3,:);

% distortion parameters
k1 = IntParam(6); k2 = IntParam(7); k3 = IntParam(8); k4 = IntParam(9); 
r = sqrt(power(norm_x, 2) + power(norm_y, 2));
theta = atan2(r,1);

% distortion
xx = (theta./r).*(1 + k1*power(theta, 2) + k2*power(theta, 4) + k3*power(theta, 6) + k4*power(theta, 8)).*norm_x;
yy = (theta./r).*(1 + k1*power(theta, 2) + k2*power(theta, 4) + k3*power(theta, 6) + k4*power(theta, 8)).*norm_y;


% If (z < 0) 
neg_z = temp(3,:) < 0; % find negative z
neg_idx = find(neg_z); % find negative z index

if ~isempty(neg_idx)
    for i=1:length(neg_idx)
        temp(3, neg_idx(i)) = -temp(3, neg_idx(i));

        norm_x = temp(1,neg_idx(i))/temp(3,neg_idx(i));
        norm_y = temp(2,neg_idx(i))/temp(3,neg_idx(i));
        r = sqrt(power(norm_x, 2) + power(norm_y, 2));
        theta = atan2(r,-1);

        xx(neg_idx(i)) = (theta/r)*(1 + k1*power(theta, 2) + k2*power(theta, 4) + k3*power(theta, 6) + k4*power(theta, 8))*norm_x;
        yy(neg_idx(i)) = (theta/r)*(1 + k1*power(theta, 2) + k2*power(theta, 4) + k3*power(theta, 6) + k4*power(theta, 8))*norm_y;
    end

end

end