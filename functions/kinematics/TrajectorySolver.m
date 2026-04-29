function [trajectory] = TrajectorySolver(points, max_vel, max_accel, plot)
% Function returns trajectory with specyfied maximum
%acceleration and velocity

% Determine the total number of waypoints
num_points = size(points, 1);
% Pre-allocate the output matrix for 3 joint angles (N rows x 3 columns)
joint_waypoints = zeros(num_points, 3); 

for i = 1 : num_points
    % Extract the full row correctly using the (row, column) format
    current_point = points(i, :);
    
    % Separate Translation (columns 1-3) and Rotation (columns 4-6)
    XYZ_vector = current_point(1:3);
    RPY_euler = current_point(4:6); 
    
    % Convert Euler angles to a 3x3 Rotation Matrix
    rot_mat = eul2rotm(RPY_euler);
    
    % Assemble the 4x4 Homogeneous Transformation Matrix
    ht_matrix = eye(4); 
    ht_matrix(1:3, 1:3) = rot_mat;       % Insert 3x3 rotation
    ht_matrix(1:3, 4) = transpose(XYZ_vector); % Insert 3x1 translation into 4th column
    
    % Calculate ALL joint angles (outputs an 8x6 matrix)
    all_q_targets = InverseKinematics(ht_matrix); 
    
    % Extract the specific row based on the user's chosen configuration index
    chosen_config_index = configurations(i);
    q_target = all_q_targets(chosen_config_index, :);
    
    % Store the calculated joint angles in our pre-allocated array
    joint_waypoints(i, :) = q_target; 
end

