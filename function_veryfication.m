clc
clear all

d1n = 200.0;
d4n = 1100.0 + 100.0;
d6n = 200.0 + 200.0;
a2n = 1300.0;

% Generating random pose of manipulator to perform calculations on
[testpose,joint_values] = ObtainRandomPoseNum(d1n, a2n, d4n, d6n);
display('Randomly generated Homogenous Transform matrix of end vector postion T0e: ');disp(testpose);
display('Randomly generated joint values according to Homogenous Transform matrix: ');disp(joint_values);

T0e = CalculateForwardKinematics(d1n, a2n, d4n, d6n, joint_values);
display('Homogenous Transform matrix calculated by designed function T0e:');disp(T0e);
solutions = CalculateInverseKinematics(d1n, a2n, d4n, d6n, testpose);
display('Solutions of inverse kinematics calculated by designed function:');disp(solutions);
%%
% Calculation of analitical dynamic model:
EulerLagrangeThreeLinkSym()