function [solutions] = CalculateInverseKinematics(d1n, a2n, d4n, d6n, T0e)
% This function performs calculation of inverse kinematics.
    solutions = [];
    % Preparation of necessary numerical matrix to perform calculations according scheme.
    Pn = T0e(1:3,4);
    Pwn = T0e(1:3,3) * d6n;
    Pan = Pn - Pwn;
    R0en = T0e(1:3,1:3);
    
    % Calculation and validation of first three joint variables
    three_first_joint_var = CalculateThreeFirstJointVariables(Pan, d1n, a2n, d4n);
    validated_three_first_joint_var = ValidateThreeFirstJointVariables(three_first_joint_var);

    % Extract number of solutions for first three joint variables
    [rows, columns] = size(validated_three_first_joint_var);
    
    % Iteration loop for merging solutions and solving latst three joint
    % variables. 
    for i = 1:rows
        % Kinematic decoupling step to matrix multiplication
        R03 = CalculateRotationFrom0To3(validated_three_first_joint_var(i, 1), validated_three_first_joint_var(i, 2), validated_three_first_joint_var(i, 3), d1n, a2n);
        R3e = transpose(R03) * R0en;
    
        % Calculation of three last joint variables
        three_last_joint_variables = CalculateThreeLastJointVariables(R3e);
        validated_three_last_joint_variables = ValidateThreeLastJointVariables(three_last_joint_variables);
        
        % Merging solutions into ona table with each row containing set of
        % solutions: [theta1, theta2, theta3, theta4, theta5, theta6]
        if ~isempty(validated_three_last_joint_variables)
            solutions(end+1,:) = [validated_three_first_joint_var(i,:), validated_three_last_joint_variables(1,:)];
            solutions(end+1,:) = [validated_three_first_joint_var(i,:), validated_three_last_joint_variables(2,:)];
        end
    end
end