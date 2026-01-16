function [rmse_matrix, noise_matrix, divergence_data, rmse_matrix_running] = robustness(rotor_data, control_input, time, initial_state, noise_data_base)
% ROBUSTNESS - Run robustness tests and return data for plotting
% Returns: rmse_matrix (Kalman), noise_matrix, divergence_data, rmse_matrix_running (Running)

fprintf('\n=== Running Robustness Analysis ===\n');

% Define 4 noise cases (Cases 1-4)
noise_cases = [
    0.0015, 0.01; % Case 1: Optimal
    0.003,  0.02; % Case 2: Regular
    0.015,  0.1;  % Case 3: High noise
    0.03,   0.2   % Case 4: Very high noise
];

num_cases = size(noise_cases, 1);
rmse_matrix = zeros(num_cases, 6);
rmse_matrix_running = zeros(num_cases, 6); % NEW: Store running filter RMSE
noise_matrix = noise_cases;

% Run simulations for Cases 1-4
for case_num = 1:num_cases
    noise_data.state_noise_amp = noise_cases(case_num, 1);
    noise_data.output_noise_amp = noise_cases(case_num, 2);
    
    [~, ~, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, initial_state);
    
    rmse_matrix(case_num, :) = errors.rmse_states;
    rmse_matrix_running(case_num, :) = errors.rmse_running; % NEW
    
    fprintf('Case %d RMSE (Kalman): ', case_num);
    fprintf('%.4f ', errors.rmse_states);
    fprintf('\n');
end

% Run divergence analysis
divergence_data = find_divergence_individual_states(rotor_data, control_input, time, initial_state, noise_data_base);

fprintf('\n=== Analysis Complete === \n');
end

%% ================= HELPER FUNCTION - TRACKS INDIVIDUAL STATE DIVERGENCE =================
function div_data = find_divergence_individual_states(rotor_data, control_input, time, initial_state, noise_data_base)
    base_noise = [noise_data_base.state_noise_amp, noise_data_base.output_noise_amp]; % Base noise levels
    max_multiplier = 50;
    multiplier = 0.2; 
    
    div_data.multipliers = [];
    div_data.rmse_values = [];          % Kalman RMSE history
    div_data.rmse_values_running = [];  % NEW: Running RMSE history
    
    % State names
    state_names = {'x', 'dx', 'y', 'dy', 'theta', 'dtheta'};
    display_names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    div_data.state_names = state_names;
    div_data.display_names = display_names;
    
    % Initialize divergence tracking for BOTH filters
    for i = 1:6
        % Kalman Tracking
        div_data.(['div_point_' state_names{i}]) = 0;
        div_data.(['actually_diverged_' state_names{i}]) = false;
        div_data.(['threshold_' state_names{i}]) = 0;
        
        % NEW: Running Filter Tracking
        div_data.(['div_point_running_' state_names{i}]) = 0;
        div_data.(['actually_diverged_running_' state_names{i}]) = false;
    end
    
    % Thresholds (Shared between filters)
    thresholds = [5.0, 1.0, 0.2, 0.2, 0.2, 0.1];  
    for i = 1:6
        div_data.(['threshold_' state_names{i}]) = thresholds(i);
    end
    
    fprintf('\n--- Starting Dual-Filter Divergence Analysis ---\n');
    
    iteration = 0;
    while multiplier <= max_multiplier
        iteration = iteration + 1;
        noise_data.state_noise_amp = base_noise(1) * multiplier;
        noise_data.output_noise_amp = base_noise(2) * multiplier;
        
        [~, ~, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, initial_state);
        
        div_data.multipliers(end+1) = multiplier;
        div_data.rmse_values(end+1, :) = errors.rmse_states;
        div_data.rmse_values_running(end+1, :) = errors.rmse_running; % NEW
        
        % Check EACH STATE for divergence (Kalman & Running)
        for state_idx = 1:6
            threshold = thresholds(state_idx);
            s_name = state_names{state_idx};
            
            % 1. Check Kalman
            if errors.rmse_states(state_idx) > threshold
                if ~div_data.(['actually_diverged_' s_name])
                    div_data.(['actually_diverged_' s_name]) = true;
                    div_data.(['div_point_' s_name]) = multiplier;
                    fprintf('  [Kalman]  %s diverged at %.1fx\n', display_names{state_idx}, multiplier);
                end
            end
            
            % 2. Check Running (NEW)
            if errors.rmse_running(state_idx) > threshold
                if ~div_data.(['actually_diverged_running_' s_name])
                    div_data.(['actually_diverged_running_' s_name]) = true;
                    div_data.(['div_point_running_' s_name]) = multiplier;
                    fprintf('  [Running] %s diverged at %.1fx\n', display_names{state_idx}, multiplier);
                end
            end
        end
        
        % Progress
        if mod(iteration, 10) == 0
            fprintf('  Progress: %.1f/%.1f\n', multiplier, max_multiplier);
        end
        
        % Early exit: Only if BOTH filters have failed on ALL states
        all_diverged = true;
        for state_idx = 1:6
            s_name = state_names{state_idx};
            if ~div_data.(['actually_diverged_' s_name]) || ~div_data.(['actually_diverged_running_' s_name])
                all_diverged = false;
                break;
            end
        end
        
        if all_diverged
            fprintf('  All states in both filters diverged. Stopping.\n');
            break;
        end
        
        multiplier = multiplier + 0.2;
    end
    
    % Cleanup: Set stable states to max_tested
    max_tested = max(div_data.multipliers);
    div_data.max_multiplier_tested = max_tested;
    
    for state_idx = 1:6
        s_name = state_names{state_idx};
        if ~div_data.(['actually_diverged_' s_name])
            div_data.(['div_point_' s_name]) = max_tested;
        end
        if ~div_data.(['actually_diverged_running_' s_name])
            div_data.(['div_point_running_' s_name]) = max_tested;
        end
    end
end