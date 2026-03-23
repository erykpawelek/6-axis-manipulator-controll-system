clc;
clear all;

% Definition of symbolic data
syms theta1 theta2 theta3 theta4 theta5 theta6;
d1 = 200.0;
d4 = 1100.0 + 100.0;
d6 = 200.0 + 200.0;
a2 = 1300.0;

% Calculation of Homogenous Transform matrixes
T1 = CalculationDHMatrix(theta1, d1, 0, sym(-pi/2))
T2 = CalculationDHMatrix(theta2 - sym(pi/2), 0, a2, 0)
T3 = CalculationDHMatrix(theta3, 0, 0, sym(-pi/2))
T4 = CalculationDHMatrix(theta4, d4, 0, sym(pi/2))
T5 = CalculationDHMatrix(theta5, 0, 0, sym(-pi/2))
T6 = CalculationDHMatrix(theta6, d6, 0, sym(-pi/2))

% Homogenous transform matric BASE - END EFFECTOR
T0e = T1*T2*T3*T4*T5*T6;

T0e_num = subs(T0e, [theta1, theta2, theta3, theta4, theta5, theta6], [0, 0, 0, 0, 0, 0])



