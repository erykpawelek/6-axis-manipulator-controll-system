function [three_first_joint_variables] = CalculateTreeFirstJointVariables(Pa, d1n, a2n, d4n)
% This function calculates three first joint variables 
    pax = Pa(1);
    pay = Pa(2);
    paz = Pa(3);

    % Theta 1 calculations
    theta1 = [atan2(pay, pax), atan2(-pay,-pax)];

    % Theta 3 calculations
    D = (a2n^2 + d4n^2 - (pax^2 + pay^2) - (paz - d1n)^2) / (2 * a2n * d4n);
    assert(isreal(D),'Target position of out reach');
    theta3 = [atan2(D, sqrt(1 - D^2)), atan2(D, -sqrt(1 - D^2))];

    % Theta 2 calculations
    Rpos = sqrt(pax^2 + pay^2);
    Rneg = -sqrt(pax^2 + pay^2);
    M1 = a2n - d4n * sin(theta3(1));
    N1 = d4n * cos(theta3(1));
    M2 = a2n - d4n * sin(theta3(2));
    N2 = d4n * cos(theta3(2));

    theta2 = [atan2(M1 * Rpos - N1 * (paz-d1n), N1 * Rpos + M1 * (paz - d1n)),
              atan2(M2 * Rpos - N2 * (paz-d1n), N2 * Rpos + M2 * (paz - d1n)),
              atan2(M1 * Rneg - N1 * (paz-d1n), N1 * Rneg + M1 * (paz - d1n)),
              atan2(M2 * Rneg - N2 * (paz-d1n), N2 * Rneg + M2 * (paz - d1n))];

    
    % Merging all solutions into one table
    three_first_joint_variables(1, :) = [theta1(1), theta2(1), theta3(1)];
    three_first_joint_variables(2, :) = [theta1(1), theta2(2), theta3(2)];
    three_first_joint_variables(3, :) = [theta1(2), theta2(3), theta3(1)];
    three_first_joint_variables(4, :) = [theta1(2), theta2(4), theta3(2)];
end