function [poly_coeffs, segment_times, joint_wps_simscape, global_time, pos_eval_simscape, velocities, accelerations] = TrajectorySolver(time_res, points, config, max_vel, max_accel, plot_flag, d1, a2, d4, d6)
% TrajectorySolver: Optimized quintic polynomial path planner mapped to Simscape 0-state.

    % --- CORE SETTINGS ---
    time_resolution = time_res; % Global evaluation fine time step (seconds) for plots

    % --- 1. Map Mathematics to Simscape Zero-State ---
    % We assume the first waypoint (points(1,:)) is the robot's home position.
    home_pt = points(1, :);
    home_rot_mat = eul2rotm(home_pt(4:6));
    home_ht = eye(4); 
    home_ht(1:3, 1:3) = home_rot_mat;
    home_ht(1:3, 4) = transpose(home_pt(1:3));
    
    all_home_configs = CalculateInverseKinematics(d1, a2, d4, d6, home_ht);
    
    % Safe config selection for Home Point
    num_sol_start = size(all_home_configs, 1);
    if num_sol_start == 0
        error('Workspace Error: The starting Home position is outside the reachable workspace.');
    end
    
    % Dynamic configuration check for the start point
    if config <= num_sol_start
        actual_home_config = config;
    else
        actual_home_config = 1;
        fprintf('WARNING: Home point not reachable in config %d. Falling back to config 1.\n', config);
    end
    
    % Assuming your Simscape and Math models are aligned perfectly, the offset is zero.
    % If you need the offset back, change this to: simscape_offset_vector = all_home_configs(actual_home_config, :);
    simscape_offset_vector = zeros(1, 6); 

    % --- 2. Calculate Mapped Inverse Kinematics ---
    num_points = size(points, 1);
    joint_wps_simscape = zeros(num_points, 6); 
    
    for i = 1 : num_points
        current_point = points(i, :);
        rot_mat = eul2rotm(current_point(4:6));
        ht_matrix = eye(4); 
        ht_matrix(1:3, 1:3) = rot_mat;
        ht_matrix(1:3, 4) = transpose(current_point(1:3));
        
        all_configs = CalculateInverseKinematics(d1, a2, d4, d6, ht_matrix); 
        num_sols = size(all_configs, 1);
        
        % Hard error: Point is physically unreachable in any configuration
        if num_sols == 0
            error('Workspace Error: Waypoint %d is unreachable in ALL configurations.', i);
        end
        
        % Dynamic Fallback Logic: Try requested config, otherwise pick the first available
        if config <= num_sols
            actual_config = config;
        else
            actual_config = 1;
            fprintf('WARNING: Waypoint %d is not reachable in requested config %d. Falling back to config 1.\n', i, config);
        end
        
        q_target_raw = all_configs(actual_config, :);
        
        % Map the raw math angle to the Simscape 0-based angle
        joint_wps_simscape(i, :) = q_target_raw - simscape_offset_vector; 
    end

    % --- 3. Time Allocation ---
    t_min = zeros(1, num_points - 1);
    for i = 2 : num_points
        % Calculate time based on the largest joint movement in this segment
        maximal_displacement = max(abs(joint_wps_simscape(i, :) - joint_wps_simscape(i-1, :)));
        t_v = 1.875 * (maximal_displacement/max_vel);
        t_a = sqrt(5.77 * maximal_displacement/max_accel);
        t_min(i-1) = max(t_v, t_a);
    end
    segment_times = t_min;
    num_segments = num_points - 1;
    total_time = sum(segment_times);

    % --- 4. Coefficient Solver ---
    num_coeffs = 6 * num_segments; 
    poly_coeffs = zeros(num_segments, 6, 6);
    
    for j = 1 : 6
        A = zeros(num_coeffs, num_coeffs);
        B = zeros(num_coeffs, 1);
        row = 1; 
        
        % Start Point (t=0) - Now starting exactly at mapped start
        A(row, 1) = 1; B(row, 1) = joint_wps_simscape(1, j); row = row + 1;
        A(row, 2) = 1; B(row, 1) = 0;                        row = row + 1;
        A(row, 3) = 2; B(row, 1) = 0;                        row = row + 1;
        
        % Via Points
        for seg = 1 : (num_segments - 1)
            T_cur = segment_times(seg); 
            c1 = (seg - 1) * 6; c2 = seg * 6;       
            q_via = joint_wps_simscape(seg + 1, j);
            
            % Position Continuity
            A(row, c1+1) = 1; A(row, c1+2) = T_cur; A(row, c1+3) = T_cur^2; 
            A(row, c1+4) = T_cur^3; A(row, c1+5) = T_cur^4; A(row, c1+6) = T_cur^5;
            B(row, 1) = q_via; row = row + 1;
            
            A(row, c2+1) = 1; B(row, 1) = q_via; row = row + 1;
            
            % Vel, Accel, Jerk, Snap Continuity
            A(row, c1+2) = 1; A(row, c1+3) = 2*T_cur; A(row, c1+4) = 3*T_cur^2; A(row, c1+5) = 4*T_cur^3; A(row, c1+6) = 5*T_cur^4; 
            A(row, c2+2) = -1; B(row, 1) = 0; row = row + 1;
            
            A(row, c1+3) = 2; A(row, c1+4) = 6*T_cur; A(row, c1+5) = 12*T_cur^2; A(row, c1+6) = 20*T_cur^3;
            A(row, c2+3) = -2; B(row, 1) = 0; row = row + 1;
            
            A(row, c1+4) = 6; A(row, c1+5) = 24*T_cur; A(row, c1+6) = 60*T_cur^2;
            A(row, c2+4) = -6; B(row, 1) = 0; row = row + 1;
            
            A(row, c1+5) = 24; A(row, c1+6) = 120*T_cur;
            A(row, c2+5) = -24; B(row, 1) = 0; row = row + 1;
        end
        
        % End Point
        T_last = segment_times(num_segments);
        c_last = (num_segments - 1) * 6; 
        
        A(row, c_last+1) = 1; A(row, c_last+2) = T_last; A(row, c_last+3) = T_last^2; 
        A(row, c_last+4) = T_last^3; A(row, c_last+5) = T_last^4; A(row, c_last+6) = T_last^5;
        B(row, 1) = joint_wps_simscape(end, j); row = row + 1;
        
        A(row, c_last+2) = 1; A(row, c_last+3) = 2*T_last; A(row, c_last+4) = 3*T_last^2; 
        A(row, c_last+5) = 4*T_last^3; A(row, c_last+6) = 5*T_last^4;
        B(row, 1) = 0; row = row + 1;
        
        A(row, c_last+3) = 2; A(row, c_last+4) = 6*T_last; A(row, c_last+5) = 12*T_last^2; A(row, c_last+6) = 20*T_last^3;
        B(row, 1) = 0;
        
        % Solve Matrix
        x = A \ B;
        for seg = 1 : num_segments
            poly_coeffs(seg, :, j) = x( ((seg-1)*6 + 1) : (seg*6) );
        end
    end 

    % --- 5. HIGH-RESOLUTION GLOBAL EVALUATION ---
    global_time = linspace(0, total_time, total_time / time_resolution);
    
    pos_eval_simscape = zeros(length(global_time), 6);
    velocities   = zeros(length(global_time), 6);
    accelerations = zeros(length(global_time), 6);
    path3D_continuous = zeros(length(global_time), 3);
    
    cumulative_seg_time = 0;
    current_segment = 1;
    
    for i = 1:length(global_time)
        t_global = global_time(i);
        
        if t_global > (cumulative_seg_time + segment_times(current_segment))
            if current_segment < num_segments
                cumulative_seg_time = cumulative_seg_time + segment_times(current_segment);
                current_segment = current_segment + 1;
            else
                t_global = total_time;
            end
        end
        
        t_local = t_global - cumulative_seg_time;
        t_local = min(max(t_local, 0), segment_times(current_segment)); 
        continuous_q_simscape_vector = zeros(1,6);
        
        for j = 1:6
            coeffs = poly_coeffs(current_segment, :, j);
            
            % These positions are now natively in the Simscape coordinate system
            pos_eval_simscape(i, j) = coeffs(1) + coeffs(2)*t_local + coeffs(3)*t_local^2 + ...
                                      coeffs(4)*t_local^3 + coeffs(5)*t_local^4 + coeffs(6)*t_local^5;
            
            continuous_q_simscape_vector(j) = pos_eval_simscape(i, j);
            
            velocities(i, j)   = coeffs(2) + 2*coeffs(3)*t_local + 3*coeffs(4)*t_local^2 + ...
                                 4*coeffs(5)*t_local^3 + 5*coeffs(6)*t_local^4;
            
            accelerations(i, j) = 2*coeffs(3) + 6*coeffs(4)*t_local + 12*coeffs(5)*t_local^2 + ...
                                 20*coeffs(6)*t_local^3;
        end
        
        % Extract X, Y, Z from the single 4x4 output of CalculateForwardKinematics
        continuous_q_raw_math = continuous_q_simscape_vector + simscape_offset_vector;
        T_matrix = CalculateForwardKinematics(d1, a2, d4, d6, continuous_q_raw_math);
        current_XYZ = T_matrix(1:3, 4)'; 
        
        path3D_continuous(i, :) = current_XYZ;
    end

    % --- 6. Plotting Section ---
    if plot_flag
        standard_linewidth = 1.5;
        joint_colors = ['b', 'r', 'g', 'm', 'c', 'y'];
        
        % Figure 1: 3D Path
        figure('Name', 'Continuous 3D Cartesian Trajectory', 'NumberTitle', 'off');
        axis equal; hold on; grid on; grid minor; view(3);
        plot3(points(:, 1), points(:, 2), points(:, 3), 'r*', 'MarkerSize', 8, 'DisplayName', 'Original Waypoints');
        plot3(path3D_continuous(:, 1), path3D_continuous(:, 2), path3D_continuous(:, 3), 'LineWidth', 2, 'Color', 'b', 'DisplayName', 'Fluent Continuous Path');
        title('Interpolated continuous 3D Path'); xlabel('X Axis'); ylabel('Y Axis'); zlabel('Z Axis'); legend('show'); hold off;
        
        % Figure 2: Simscape Joint Displacements 
        figure('Name', 'Native Simscape Joint Displacements', 'NumberTitle', 'off');
        hold on; grid on; grid minor;
        for j = 1:6
            plot(global_time, pos_eval_simscape(:, j), 'Color', joint_colors(j), 'LineWidth', standard_linewidth, 'DisplayName', ['Joint ', num2str(j)]);
        end
        cumulative_times_vec = [0, cumsum(segment_times)];
        for j = 1:6
            plot(cumulative_times_vec, joint_wps_simscape(:, j), [joint_colors(j), 'o'], 'MarkerFaceColor', 'w', 'MarkerSize', 5, 'HandleVisibility', 'off');
        end
        title('Joint Displacements (0 = Simscape Home Position)');
        xlabel('Time (s)'); ylabel('Angle (rad)'); legend('show'); hold off;
        
        % Figure 3: Velocities
        figure('Name', 'Joint Velocities', 'NumberTitle', 'off');
        hold on; grid on; grid minor;
        for j = 1:6
            plot(global_time, velocities(:, j), 'Color', joint_colors(j), 'LineWidth', standard_linewidth, 'DisplayName', ['Joint ', num2str(j)]);
        end
        title('Smooth continuous joint velocity profiles'); xlabel('Time (s)'); ylabel('Velocity (rad/s)');legend('show'); hold off;
        
        % Figure 4: Accelerations
        figure('Name', 'Joint Accelerations', 'NumberTitle', 'off');
        hold on; grid on; grid minor;
        for j = 1:6
            plot(global_time, accelerations(:, j), 'Color', joint_colors(j), 'LineWidth', standard_linewidth, 'DisplayName', ['Joint ', num2str(j)]);
        end
        title('Smooth continuous joint acceleration profiles'); xlabel('Time (s)'); ylabel('Acceleration (rad/s^2)');legend('show'); hold off;
    end
end