%% Master Trajectory Planner (Hybrid Joint & Cartesian Space)
clc;
clear;
close all;

% 1. Kinematic Parameters
d1n = 200.0;
d4n = 1100.0 + 100.0;
d6n = 200.0 + 200.0;
a2n = 1300.0;
time_res = 0.0001;

% 2. Waypoint Definitions [X, Y, Z, Roll, Pitch, Yaw]
p1 = [  1600, 0, 1500, -pi/2, -pi/2, -pi/2]; % Home Start
p2 = [ -1800, 0,  400,   -pi,     0,   -pi]; % Pre-approach (Above target)
p3 = [ -1800, 0,  200,   -pi,     0,   -pi]; % Target (Insert down)
p4 = [ -1800, 0,  500,   -pi,     0,   -pi]; % Retract (Pull up)
p5 = [  1600, -282.8, 817, -pi/2, -pi/4, -pi]; % Start of Approach (Staging Point)

% Add new points for pickup and retract using explicit cell geometry
dist_approach = -400.0;
p6 = [p5(1), p5(2) + dist_approach*cos(pi/4), p5(3) + dist_approach*sin(pi/4), p5(4), p5(5), p5(6)]; % Pickup Point
dist_retract = 200.0;
p7 = [p5(1), p5(2) + dist_retract*cos(pi/4), p5(3) + dist_retract*sin(pi/4), p5(4), p5(5), p5(6)]; % Retract Point

% 3. Setup Solver Limits
ik_config = 1; 

% Joint Space Limits (Sweeping moves)
max_joint_velocity = pi;       % Maximum velocity in rad/s
max_joint_acceleration = 0.15*pi; % Maximum acceleration in rad/s^2

% Cartesian Space Limits (Straight-line approaches)
max_linear_velocity = 100;      % Maximum velocity in mm/s
max_linear_acceleration = 1008;  % Maximum acceleration in mm/s^2

enable_plotting = false; % Turn off internal solver plots to avoid clutter

%% --- EXECUTE TRAJECTORY MODULES ---
disp('Calculating Segment 1: Joint Space (Home to Pre-Approach)...');
[~, ~, ~, t1, pos1, vel1, acc1] = TrajectorySolver(time_res, [p1; p2], ik_config, max_joint_velocity, max_joint_acceleration, enable_plotting, d1n, a2n, d4n, d6n);

disp('Calculating Segment 2: Cartesian Space (Linear Insert)...');
[~, t2, pos2, vel2, acc2] = LinearTrajectorySolver(time_res, p2, p3, ik_config, max_linear_velocity, max_linear_acceleration, d1n, a2n, d4n, d6n);

disp('Calculating Segment 3: Cartesian Space (Linear Retract)...');
[~, t3, pos3, vel3, acc3] = LinearTrajectorySolver(time_res, p3, p4, ik_config, max_linear_velocity, max_linear_acceleration, d1n, a2n, d4n, d6n);

disp('Calculating Segment 4: Joint Space (Sweep to Staging Point)...');
[~, ~, ~, t4, pos4, vel4, acc4] = TrajectorySolver(time_res, [p4; p5], ik_config, max_joint_velocity, max_joint_acceleration, enable_plotting, d1n, a2n, d4n, d6n);

disp('Calculating Segment 5: Cartesian Space (Linear Insert to Pickup)...');
[~, t5, pos5, vel5, acc5] = LinearTrajectorySolver(time_res, p5, p6, ik_config, max_linear_velocity, max_linear_acceleration, d1n, a2n, d4n, d6n);

disp('Calculating Segment 6: Cartesian Space (Linear Retract from Pickup)...');
[~, t6, pos6, vel6, acc6] = LinearTrajectorySolver(time_res, p6, p7, ik_config, max_linear_velocity, max_linear_acceleration, d1n, a2n, d4n, d6n);

disp('Calculating Segment 7: Joint Space (Return to Home)...');
[~, ~, ~, t7, pos7, vel7, acc7] = TrajectorySolver(time_res, [p7; p1], ik_config, max_joint_velocity, max_joint_acceleration, enable_plotting, d1n, a2n, d4n, d6n);

%% --- SEAMLESS STITCHING ---
disp('Stitching trajectory segments...');
% Shift the time vectors so they flow continuously. 
t2_shifted = t2(2:end) + t1(end); 
t3_shifted = t3(2:end) + t2_shifted(end);
t4_shifted = t4(2:end) + t3_shifted(end);
t5_shifted = t5(2:end) + t4_shifted(end);
t6_shifted = t6(2:end) + t5_shifted(end);
t7_shifted = t7(2:end) + t6_shifted(end);

% Concatenate all time vectors into one continuous column
master_time = [t1, t2_shifted, t3_shifted, t4_shifted, t5_shifted, t6_shifted, t7_shifted]'; 

% Concatenate all kinematic matrices (vertically stack them)
master_pos = [pos1; pos2(2:end, :); pos3(2:end, :); pos4(2:end, :); pos5(2:end, :); pos6(2:end, :); pos7(2:end, :)];
master_vel = [vel1; vel2(2:end, :); vel3(2:end, :); vel4(2:end, :); vel5(2:end, :); vel6(2:end, :); vel7(2:end, :)];
master_acc = [acc1; acc2(2:end, :); acc3(2:end, :); acc4(2:end, :); acc5(2:end, :); acc6(2:end, :); acc7(2:end, :)];

%% --- PACKAGE FOR SIMSCAPE ---
% Slice the master matrices by column to isolate each joint
sim_input_J1 = [master_time, master_pos(:,1), master_vel(:,1), master_acc(:,1)];
sim_input_J2 = [master_time, master_pos(:,2), master_vel(:,2), master_acc(:,2)];
sim_input_J3 = [master_time, master_pos(:,3), master_vel(:,3), master_acc(:,3)];
sim_input_J4 = [master_time, master_pos(:,4), master_vel(:,4), master_acc(:,4)];
sim_input_J5 = [master_time, master_pos(:,5), master_vel(:,5), master_acc(:,5)];
sim_input_J6 = [master_time, master_pos(:,6), master_vel(:,6), master_acc(:,6)];

disp('Simulink data packaged successfully!');
disp(['Total continuous movement time: ', num2str(master_time(end)), ' seconds']);

%% --- MASTER PLOTTING SECTION ---
disp('Calculating Forward Kinematics for 3D visualization...');

% Retrieve the total number of evaluation steps calculated during stitching
num_total_steps = size(master_pos, 1);
path3D_master = zeros(num_total_steps, 3);

for i = 1:num_total_steps
    % Run each joint configuration through Forward Kinematics to find the XYZ tool position
    T_matrix = CalculateForwardKinematics(d1n, a2n, d4n, d6n, master_pos(i, :));
    path3D_master(i, :) = T_matrix(1:3, 4)';
end

disp('Generating plots...');
joint_colors = ['b', 'r', 'g', 'm', 'c', 'y'];
standard_linewidth = 1.5;

% Create Figure 1: 3D Cartesian Path
figure('Name', 'Master 3D Cartesian Path', 'NumberTitle', 'off');
hold on; grid on; grid minor; view(3);

% Plot the continuous line calculated from the stitched joint angles
plot3(path3D_master(:,1), path3D_master(:,2), path3D_master(:,3), 'g-', 'LineWidth', 2, 'DisplayName', 'Executed Path');

% Plot the explicit waypoints for visual reference
waypoints = [p1; p2; p3; p4; p5; p6; p7];
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Waypoints');

xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Complete Stitched 3D Trajectory');
legend('show');
hold off;

% Create Figure 2: Master Joint Kinematics Window
figure;

hold on; grid on; grid minor;
for j = 1:6
    plot(master_time, master_pos(:, j), 'Color', joint_colors(j), 'LineWidth', standard_linewidth, 'DisplayName', ['Joint ', num2str(j)]);
end
ylabel('Position (rad)');
title('Joint Displacements');
legend('show', 'Location', 'eastoutside');
hold off;

% Plot 2: Joint Velocities
figure;
hold on; grid on; grid minor;
for j = 1:6
    plot(master_time, master_vel(:, j), 'Color', joint_colors(j), 'LineWidth', standard_linewidth, 'DisplayName', ['Joint ', num2str(j)]);
end
ylabel('Velocity (rad/s)');
title('Joint Velocities');
hold off;

% Plot 3: Joint Accelerations
figure;
hold on; grid on; grid minor;
for j = 1:6
    plot(master_time, master_acc(:, j), 'Color', joint_colors(j), 'LineWidth', standard_linewidth, 'DisplayName', ['Joint ', num2str(j)]);
end
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
title('Joint Accelerations');
hold off;