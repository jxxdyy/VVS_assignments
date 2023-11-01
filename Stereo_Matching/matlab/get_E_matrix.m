function [E] = get_E_matrix(F, l_camK, r_camK)
%GET_E_MATRIX Summary of this function goes here
%   Detailed explanation goes here

E = r_camK'*F*l_camK;

end

