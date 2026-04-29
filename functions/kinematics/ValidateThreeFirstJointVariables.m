function [validated_three_first_joint_variables] = ValidateThreeFirstJointVariables(three_first_joint_variables)
% This function filters the 4 spatial configurations using logical indexing and performs angle unwrapping.

    theta2_col = three_first_joint_variables(:, 2);
    theta3_col = three_first_joint_variables(:, 3);

    % Mask preparations 4x1
    theta3_wrap_mask = (theta3_col > pi/3);
    theta3_col(theta3_wrap_mask) = theta3_col(theta3_wrap_mask) - 2*pi;
    is_real_mask = all(imag(three_first_joint_variables) == 0, 2);
    theta3_limit_mask = (theta3_col <= pi/3) & (theta3_col >= (-240/360 * 2*pi));
    theta2_limit_mask = (theta2_col <= pi/3) & (theta2_col >= -pi/3);

    % Unwraping of theta3 angles
    three_first_joint_variables(theta3_wrap_mask, 3) = theta3_col(theta3_wrap_mask);
    
    % Combine all conditions into a single master mask for the rows
    master_valid_mask = is_real_mask & theta3_limit_mask & theta2_limit_mask;
    
    % Extract and return only the rows where the master mask is true (1)
    validated_three_first_joint_variables = three_first_joint_variables(master_valid_mask, :);
end