function [T0e_num, joint_angles] = ObtainRandomPoseNum(d1, a2, d4, d6)
% This function generates random pose of manipulator.

    % Theta 1: +/- 180 deg (-pi to pi)
    % Range = 2*pi, Min = -pi
    theta1_num = (rand() * 2 * pi) - pi;
    
    % Theta 2: +/- 60 deg (-pi/3 to pi/3)
    % Range = 2*pi/3, Min = -pi/3
    theta2_num = (rand() * (2 * pi / 3)) - (pi / 3);
    
    % Theta 3: +60 to -240 deg (-4*pi/3 to pi/3)
    % Range = 5*pi/3, Min = -4*pi/3
    theta3_num = (rand() * (5 * pi / 3)) - (4 * pi / 3);
    
    % Theta 4: +/- 180 deg (-pi to pi)
    % Range = 2*pi, Min = -pi
    theta4_num = (rand() * 2 * pi) - pi;
    
    % Theta 5: +/- 90 deg (-pi/2 to pi/2)
    % Range = pi, Min = -pi/2
    theta5_num = (rand() * pi) - (pi / 2);
    
    % Theta 6: +/- 180 deg (-pi to pi)
    % Range = 2*pi, Min = -pi
    theta6_num = (rand() * 2 * pi) - pi;

    joint_angles = [theta1_num, theta2_num, theta3_num, theta4_num, theta5_num, theta6_num];

    % Defining Homogeneous Transform pose representation
    T1 = CalculateDHMatrix(theta1_num, d1, 0, -pi/2);
    T2 = CalculateDHMatrix(theta2_num - pi/2, 0, a2, 0);
    T3 = CalculateDHMatrix(theta3_num, 0, 0, -pi/2);
    T4 = CalculateDHMatrix(theta4_num, d4, 0, pi/2);
    T5 = CalculateDHMatrix(theta5_num, 0, 0, -pi/2);
    T6 = CalculateDHMatrix(theta6_num, d6, 0, 0);

    % Calculation of the final Homogeneous Transform matrix (BASE to END EFFECTOR)
    T0e_num = T1 * T2 * T3 * T4 * T5 * T6;
end