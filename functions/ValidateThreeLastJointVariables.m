function [validated_three_last_joint_variables] = ValidateThreeLastJointVariables(three_last_joint_variables)
% This function verifies values of three last joint variable sets

% Exxtract theta5 column
theta5_col = three_last_joint_variables(:,2);

% Mask for checking range of motion
theta5_mask = theta5_col <= pi/2 & theta5_col >= -pi/2;

% Final set of validated solutions
validated_three_last_joint_variables = three_last_joint_variables(theta5_mask, :);
end