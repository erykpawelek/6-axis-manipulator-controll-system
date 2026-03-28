function [tree_last_joint_variables] = CalculateThreeLastJointVariables(R3en)
% This function calculates joints variables of spherical wrist.

% Extracted values of rotation matrix
R13 = R3en(1, 3);
R23 = R3en(2, 3);
R33 = R3en(3, 3);
R31 = R3en(3, 1);
R32 = R3en(3, 2);

% Theta 5 calculations
theta5 = [atan2(sqrt(R13^2 + R23^2), R33), atan2(-sqrt(R13^2 + R23^2), R33)];

% Theta 4 calculations
theta4 = [atan2(-R23, -R13), atan2(R23, R13)];

% Theta 6 calculations
theta6 = [atan2(-R32, R31), atan2(R32, -R31)];

% Merging solutions into one table
tree_last_joint_variables(1,:) = [theta4(1), theta5(1), theta6(1)];
tree_last_joint_variables(2,:) = [theta4(2), theta5(2), theta6(2)];

tree_last_joint_variables = double(tree_last_joint_variables);
end