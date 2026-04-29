function [R03] = CalculateRotationFrom0To3(theta1, theta2, theta3, d1n, a2n) 
% This function calculates symbolic form of homogeneus matrix from .

    % Calculation of Homogenous Transform matrixes
    T1 = CalculateDHMatrix(theta1, d1n, 0, -pi/2);
    T2 = CalculateDHMatrix(theta2 - pi/2, 0, a2n, 0);
    T3 = CalculateDHMatrix(theta3, 0, 0, -pi/2);

    % Homogenous transform matric BASE - END EFFECTOR
    T03 = T1*T2*T3;
    R03 = T03(1:3,1:3);
end