%% Manipulator parameters determination
clc
clear all

syms theta1 theta2 theta3 theta4 theta5 theta6 d1 a2 d4 d6;

d1n = 200.0;
d4n = 1100.0 + 100.0;
d6n = 200.0 + 200.0;
a2n = 1300.0;

%% Forvard kinematics 
T0e = ObtainGeometricalModelSym();
display('Geometrical model of analised manipulator:'); disp(T0e);

% Values of joint variables
theta1n = 0;
theta2n = 0;
theta3n = 0;
theta4n = 0;
theta5n = 0;
theta6n = 0;

% Numerical representation of T0e matrix
T0e_num = double(subs(T0e, {theta1, theta2, theta3, theta4, theta5, theta6, d1, a2, d4, d6}, {theta1n, theta2n, theta3n, theta4n, theta5n, theta6n, d1n, a2n, d4n, d6n}));

display('Numerical representation of transformation matrix T0e_num for given joint variables:'); disp(T0e_num);

%% Inverse kinematics 

% 1 Step of kinematic decoupling method
P = T0e(1:3,4);
Pw = T0e(1:3,3) * d6;
Pa = simplify(P - Pw);

display('End effector position vector:'); disp(P);
display('Z axis orientation of end effector'); disp(Pw);
display('Pa vector'); disp(Pa);

% Generating random pose of manipulator to perform calculations on
[testpose,joint_values] = ObtainRandomPoseNum(d1n, a2n, d4n, d6n);
Pn = testpose(1:3,4);
Pwn = testpose(1:3,3) * d6n;
Pan = Pn - Pwn;
display('Joint variables of robots tested configuration:')
disp(joint_values);

% Calculation of first 3 joint variables;
three_first_joint_var = CalculateThreeFirstJointVariables(Pan, d1n, a2n, d4n)
validated_three_first_joint_var = ValidateThreeFirstJointVariables(three_first_joint_var)

% Calculation of last 3 joint variables;

[rows, columns] = size(validated_three_first_joint_var);

T3e = CalculateTranformFrom3ToESymbolic()
R3e = T3e(1:3, 1:3)
R0e = testpose(1:3,1:3);


for i = 1:rows
    R03 = CalculateRotationFrom0To3(validated_three_first_joint_var(i, 1), validated_three_first_joint_var(i, 2), validated_three_first_joint_var(i, 3), d1n, a2n);
    R3e = transpose(R03) * R0e;

    three_lats_joint_variables(i) = CalculateThreeLastJointVariables(R3e) 
end