function [T3e_sym] = CalculateTranformFrom3ToESymbolic() 
% This function calculates symbolic form of homogeneus matrix from .

    % Definition of symbolic data
    syms theta1 theta2 theta3 theta4 theta5 theta6;
    syms d1 a2 d4 d6

    % Calculation of Homogenous Transform matrixes
    T4 = CalculateDHMatrix(theta4, d4, 0, sym(pi/2));
    T5 = CalculateDHMatrix(theta5, 0, 0, sym(-pi/2));
    T6 = CalculateDHMatrix(theta6, d6, 0, 0);

    % Homogenous transform matric BASE - END EFFECTOR
    T3e_sym = T4*T5*T6;
end