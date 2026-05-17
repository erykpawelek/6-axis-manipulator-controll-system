function [joint_wps_simscape, global_time, pos_eval_simscape, velocities, accelerations] = LinearTrajectorySolver(time_res, P_start, P_end, config, max_linear_vel, max_linear_accel, d1, a2, d4, d6)
% LinearTrajectorySolver: Cartesian straight-line planner with Trapezoidal velocity.

    % --- 1. Cartesian Path Geometry ---
    % Calculate the total linear distance between start and end XYZ coordinates
    vec_distance = P_end(1:3) - P_start(1:3);
    D = norm(vec_distance); % Absolute distance in mm
    
    if D == 0
        error('Start and End points are identical. No movement required.');
    end

    % --- 2. Trapezoidal Velocity Profile (LSPB) ---
    % Calculate the time required to accelerate to max velocity
    t_blend = max_linear_vel / max_linear_accel;
    
    % Calculate the distance covered during the acceleration phase
    d_blend = 0.5 * max_linear_accel * t_blend^2;
    
    % Check if the distance is too short to ever reach max velocity (Triangular profile)
    if D < 2 * d_blend
        % Recalculate max velocity and blend time for a triangular profile
        max_linear_vel = sqrt(D * max_linear_accel);
        t_blend = max_linear_vel / max_linear_accel;
        t_constant = 0;
    else
        % Calculate how long the robot will cruise at constant max velocity
        d_constant = D - (2 * d_blend);
        t_constant = d_constant / max_linear_vel;
    end
    
    total_time = (2 * t_blend) + t_constant;
    global_time = linspace(0, total_time, round(total_time / time_res));
    num_steps = length(global_time);
    
    % Pre-allocate arrays for the Cartesian distance 's' along the line
    s_t = zeros(num_steps, 1);
    
    % Calculate the exact position along the line (0 to D) for every time step
    for i = 1:num_steps
        t = global_time(i);
        if t <= t_blend
            % Acceleration Phase
            s_t(i) = 0.5 * max_linear_accel * t^2;
        elseif t > t_blend && t <= (t_blend + t_constant)
            % Constant Velocity Phase
            s_t(i) = d_blend + max_linear_vel * (t - t_blend);
        else
            % Deceleration Phase
            t_dec = t - t_blend - t_constant;
            s_t(i) = (D - d_blend) + (max_linear_vel * t_dec) - (0.5 * max_linear_accel * t_dec^2);
        end
    end

    % --- 3. High-Resolution Inverse Kinematics Loop ---
    pos_eval_simscape = zeros(num_steps, 6);
    joint_wps_simscape = zeros(2, 6); % Store just the start and end joint angles
    
    disp('Calculating high-resolution Cartesian inverse kinematics...');
    
    for i = 1:num_steps
        % Interpolate XYZ coordinates based on the current distance s_t
        % Formula: Start + (Percentage of total distance) * Total Vector
        current_XYZ = P_start(1:3) + (s_t(i) / D) * vec_distance;
        
        % Linearly interpolate the Roll, Pitch, Yaw angles
        current_RPY = P_start(4:6) + (s_t(i) / D) * (P_end(4:6) - P_start(4:6));
        
        % Build Homogeneous Transformation Matrix
        rot_mat = eul2rotm(current_RPY);
        ht_matrix = eye(4);
        ht_matrix(1:3, 1:3) = rot_mat;
        ht_matrix(1:3, 4) = current_XYZ';
        
        % Calculate Inverse Kinematics
        all_configs = CalculateInverseKinematics(d1, a2, d4, d6, ht_matrix);
        num_sols = size(all_configs, 1);
        
        if num_sols == 0
            error('Cartesian Path Error: The straight line leaves the reachable workspace at step %d.', i);
        end
        
        % Dynamic Configuration Fallback
        if config <= num_sols
            actual_config = config;
        else
            actual_config = 1;
            if i == 1
                fprintf('WARNING: Requested config %d not available. Falling back to config 1.\n', config);
            end
        end
        
        pos_eval_simscape(i, :) = all_configs(actual_config, :);
        
        % Save Start and End points for reference
        if i == 1
            joint_wps_simscape(1, :) = pos_eval_simscape(1, :);
        elseif i == num_steps
            joint_wps_simscape(2, :) = pos_eval_simscape(end, :);
        end
    end
    
    % --- 4. Numerical Joint Velocities and Accelerations ---
    velocities = zeros(num_steps, 6);
    accelerations = zeros(num_steps, 6);
    
    for j = 1:6
        % Calculate velocity as the derivative of position over time
        velocities(:, j) = gradient(pos_eval_simscape(:, j), time_res);
        % Calculate acceleration as the derivative of velocity over time
        accelerations(:, j) = gradient(velocities(:, j), time_res);
    end
    
    disp('Linear Cartesian trajectory calculation complete!');
end