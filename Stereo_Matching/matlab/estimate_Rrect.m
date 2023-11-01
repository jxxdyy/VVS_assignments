function [Rrect1,Rrect2] = estimate_Rrect(t, R)
%UNTITLED Summary of this function goes here

%   Detailed explanation goes here

e1 = t/norm(t);
e2 = (1/norm(t(1:2)))*[-t(2) t(1) 0]';
e3 = cross(e1,e2);
R_rect = [e1'; e2'; e3'];

Rrect1 = R_rect;
Rrect2 = R*R_rect;

end

