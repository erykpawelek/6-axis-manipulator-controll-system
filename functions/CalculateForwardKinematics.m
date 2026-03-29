function [T0e_num] = CalculateForwardKinematics(d1n, a2n, d4n, d6n, joint_angles)
% This function calculates forward kinematics of the manipulator;

    % Preparations of Homogenous Transform matrixes
    T1 = CalculateDHMatrix(joint_angles(1), d1n, 0, -pi/2);
    T2 = CalculateDHMatrix(joint_angles(2) - pi/2, 0, a2n, 0);
    T3 = CalculateDHMatrix(joint_angles(3), 0, 0, -pi/2);
    T4 = CalculateDHMatrix(joint_angles(4), d4n, 0, pi/2);
    T5 = CalculateDHMatrix(joint_angles(5), 0, 0, -pi/2);
    T6 = CalculateDHMatrix(joint_angles(6), d6n, 0, 0);

    % Homoheneous transform from 0 to end effector
    T0e_num = T1 * T2 * T3 * T4 * T5 * T6;
end