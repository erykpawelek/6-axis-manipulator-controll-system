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

joint_values = [0 0 0 0 0 0]
T0e = CalculateForwardKinematics(d1n, a2n, d4n, d6n, joint_values);
display('Homogenous Transform matrix calculated by designed function T0e:');disp(T0e);
solutions = CalculateInverseKinematics(d1n, a2n, d4n, d6n, testpose);
display('Solutions of inverse kinematics calculated by designed function:');disp(solutions);
%%
% Calculation of analitical dynamic model:
EulerLagrangeThreeLinkSym()
%%
clc;
clear;
close all;
d1n = 200.0;
d4n = 1100.0 + 100.0;
d6n = 200.0 + 200.0;
a2n = 1300.0;
time_res = 0.001;

% Format: [X, Y, Z, Roll, Pitch, Yaw]
p1 = [ 1600,    0,  1500,  -1.571, -1.571, -1.571]; % Start point
p2 = [ 1600,  200,  1600,  -1.571, -1.571, -1.571]; % Via-point 1
p3 = [ 1600,  300,  1700,  -1.571, -1.571, -1.571]; % Via-point 2
p4 = [ 1400, -100,  1700,  -1.571, -1.571, -1.571]; % Via-point 2
p5 = [ 1800, -200,  1600,  -1.300, -1.571, -1.571]; % Via-point 2
p6 = [ 1600,  300,  1700,  -1.571, -1.571, -1.571]; % Via-point 2
p7 = [ 1600,    0,  1500,  -1.571, -1.571, -1.571]; % End point

% Combine individual points into a single Nx6 path matrix
path_points = [p1; p2; p3; p4; p5; p6; p7];

% 2. Setup Solver Parameters
% Select which inverse kinematics solution to use (1 through 8)
ik_config = 1; 

% Define maximum kinematic limits for the joints
max_joint_velocity = pi;       % Maximum velocity in rad/s
max_joint_acceleration = 2*pi; % Maximum acceleration in rad/s^2

% Enable plotting to see the generated profiles
enable_plotting = true;

% 3. Execute the Trajectory Solver
disp('Calculating trajectory...');
[poly_coeffs, segment_times, joint_wps_simscape, global_time, pos_eval_simscape, velocities, accelerations] = TrajectorySolver(...
    time_res,...
    path_points, ...
    ik_config, ...
    max_joint_velocity, ...
    max_joint_acceleration, ...
    enable_plotting, ...
    d1n, ...
    a2n, ...
    d4n, ...
    d6n);

% 4. Output basic statistics to the console
disp('Trajectory calculation complete!');
disp('Total movement time (seconds):');
disp(sum(segment_times));

t_col = global_time'; % Transpose global_time from a row to a column vector

% Package [Time, Position, Velocity, Acceleration] for each specific joint
sim_input_J1 = [t_col, pos_eval_simscape(:,1), velocities(:,1), accelerations(:,1)];
sim_input_J2 = [t_col, pos_eval_simscape(:,2), velocities(:,2), accelerations(:,2)];
sim_input_J3 = [t_col, pos_eval_simscape(:,3), velocities(:,3), accelerations(:,3)];
sim_input_J4 = [t_col, pos_eval_simscape(:,4), velocities(:,4), accelerations(:,4)];
sim_input_J5 = [t_col, pos_eval_simscape(:,5), velocities(:,5), accelerations(:,5)];
sim_input_J6 = [t_col, pos_eval_simscape(:,6), velocities(:,6), accelerations(:,6)];

disp('Simulink data packaged successfully!');