function [trajectory] = TrajectorySolver(points, config, max_vel, max_accel, plot)
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
    q_target = all_q_targets(config, :);
    
    % Store the calculated joint angles in our pre-allocated array
    joint_waypoints(i, :) = q_target; 
end

% Calculating times between points
t = zeros(1, num_points - 1);
for i = 2 : num_points
        
    % Finding maximal radial displacement
     joint_diff = joint_waypoints(i, :) - joint_waypoints(i-1, :);
     maximal_displacement = max(abs(joint_diff));
     
     % Calculating minimal time based on derivatives of 5th order polynomials
     t_v = 1.875 * (maximal_displacement/max_vel);
     t_a = sqrt(5.77 * maximal_displacement/max_accel);

     if t_v > t_a
         t(i-1) = t_v;
     else
         t(i-1) = t_a;
     end
end

% Calculate the number of segments
num_segments = num_points - 1;

% Each segment has a 5th-order polynomial with 6 coefficients (a0 to a5)
% Total number of unknown variables for one joint
num_coeffs = 6 * num_segments; 

% Pre-allocate a 3D matrix to store the final coefficients
% Dimensions: [Rows = Segments, Columns = 6 Coefficients, Depth = 6 Joints]
polynomial_coeffs = zeros(num_segments, 6, 6);

for joint_idx = 1 : 6
    % 1. Initialize the global A matrix and B vector for this specific joint
    A = zeros(num_coeffs, num_coeffs);
    B = zeros(num_coeffs, 1);
    
    % We use a row counter to keep track of which equation we are adding to the matrix
    row = 1; 
    
    % --- Start Point Boundary Conditions (Applies only to Segment 1 at t = 0) ---
    % Equation 1: Start Position (a0 = initial angle)
    A(row, 1) = 1; 
    B(row, 1) = joint_waypoints(1, joint_idx);
    row = row + 1;
    % Equation 2: Start Velocity (a1 = 0)
    A(row, 2) = 1;
    B(row, 1) = 0;
    row = row + 1; 
    % Equation 3: Start Acceleration (2*a2 = 0)
    A(row, 3) = 2;
    B(row, 1) = 0;
    row = row + 1;
    
    % --- [Placeholder for Continuity Equations and End Point Equations] ---
   for seg = 1 : (num_segments - 1)
        % T is the total time duration for the CURRENT segment
        T = t(seg); 
        
        % Calculate the base column index for current and next segment
        c1 = (seg - 1) * 6; % Columns for Segment A (ends at via-point)
        c2 = seg * 6;       % Columns for Segment B (starts at via-point)
        
        % The target angle for this specific joint at this via-point
        target_angle = joint_waypoints(seg + 1, joint_idx);
        
        % 1. Position: Segment A ends exactly at the via-point target
        A(row, c1+1) = 1; A(row, c1+2) = T; A(row, c1+3) = T^2; A(row, c1+4) = T^3; A(row, c1+5) = T^4; A(row, c1+6) = T^5;
        B(row, 1) = target_angle;
        row = row + 1;
        
        % 2. Position: Segment B starts exactly at the via-point target (at local t=0)
        A(row, c2+1) = 1; 
        B(row, 1) = target_angle;
        row = row + 1;
        
        % 3. Velocity Continuity (End Vel A - Start Vel B = 0)
        A(row, c1+2) = 1; A(row, c1+3) = 2*T; A(row, c1+4) = 3*T^2; A(row, c1+5) = 4*T^3; A(row, c1+6) = 5*T^4; % End Vel A
        A(row, c2+2) = -1; % Start Vel B (moved to left side of equation)
        B(row, 1) = 0;
        row = row + 1;
        
        % 4. Acceleration Continuity (End Accel A - Start Accel B = 0)
        A(row, c1+3) = 2; A(row, c1+4) = 6*T; A(row, c1+5) = 12*T^2; A(row, c1+6) = 20*T^3;
        A(row, c2+3) = -2; 
        B(row, 1) = 0;
        row = row + 1;
        
        % 5. Jerk Continuity (End Jerk A - Start Jerk B = 0)
        A(row, c1+4) = 6; A(row, c1+5) = 24*T; A(row, c1+6) = 60*T^2;
        A(row, c2+4) = -6;
        B(row, 1) = 0;
        row = row + 1;
        
        % 6. Snap Continuity (End Snap A - Start Snap B = 0)
        A(row, c1+5) = 24; A(row, c1+6) = 120*T;
        A(row, c2+5) = -24;
        B(row, 1) = 0;
        row = row + 1;
    end


    end
    % 2. Solve the linear system A * x = B to find all coefficients for this joint
    % The backslash operator calculates x
    x = A \ B;
    
    % --- [Placeholder for reshaping x back into the polynomial_coeffs matrix] ---
    
end



