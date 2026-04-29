function [T] = CalculateDHMatrix(theta, d, a, alpha)
% This function calculates the 4x4 homogeneous transformation matrix using the four standard Denavit-Hartenberg parameters.

T_row1 = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)];
T_row2 = [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)];
T_row3 = [0, sin(alpha), cos(alpha), d];
T_row4 = [0, 0, 0, 1];

T = [T_row1; T_row2; T_row3; T_row4];
end