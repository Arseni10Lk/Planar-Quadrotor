function [rmse_matrix, noise_matrix, divergence_data] = robustness(rotor_data, control_input, time, initial_state)
% ROBUSTNESS - Run robustness tests and return data for plotting
% Returns: rmse_matrix (4x6), noise_matrix (4x2), divergence_data

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
noise_matrix = noise_cases;

% Run simulations for Cases 1-4
for case_num = 1:num_cases
    noise_data.state_noise_amp = noise_cases(case_num, 1);
    noise_data.output_noise_amp = noise_cases(case_num, 2);
    
    [~, ~, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, initial_state);
    
    rmse_matrix(case_num, :) = errors.rmse_states;
    fprintf('Case %d RMSE: ', case_num);
    fprintf('%.4f ', errors.rmse_states);
    fprintf('\n');
end

% Run divergence analysis
divergence_data = find_divergence_individual_states(rotor_data, control_input, time, initial_state);

fprintf('\n=== Analysis Complete === \n');
end

%% ================= HELPER FUNCTION - TRACKS INDIVIDUAL STATE DIVERGENCE =================
function div_data = find_divergence_individual_states(rotor_data, control_input, time, initial_state)
    base_noise = [0.003, 0.02]; % Base noise levels
    max_multiplier = 50;
    multiplier = 0.2; % Start from 0.2x as suggested
    
    div_data.multipliers = [];
    div_data.rmse_values = [];
    div_data.diverged = false;
    
    % State names for field creation
    state_names = {'x', 'dx', 'y', 'dy', 'theta', 'dtheta'};
    display_names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'}; % For display
    div_data.state_names = state_names;
    div_data.display_names = display_names;
    
    % Initialize divergence tracking for EACH STATE
    for i = 1:6
        div_data.(['div_point_' state_names{i}]) = 0;  % Will be updated
        div_data.(['actually_diverged_' state_names{i}]) = false; % True if exceeded threshold
        div_data.(['threshold_' state_names{i}]) = 0; % Store threshold for reference
    end
    
    % ============ CRITICAL: SET DIVERGENCE THRESHOLDS HERE ============
    % Adjust these values to match system's expected performance
    thresholds = [5.0, 1.0, 0.5, 0.2, 0.5, 0.3];  
   
    % ==================================================================
    
    for i = 1:6
        div_data.(['threshold_' state_names{i}]) = thresholds(i);
    end
    
    fprintf('\n--- Starting Individual State Divergence Analysis ---\n');
    fprintf('Testing from %.1fx to %.1fx noise multiplier\n', multiplier, max_multiplier);
    fprintf('\nCurrent divergence thresholds:\n');
    fprintf('  x:   %.1f m      y:   %.1f m\n', thresholds(1), thresholds(3));
    fprintf('  dx:  %.1f m/s    dy:  %.1f m/s\n', thresholds(2), thresholds(4));
    fprintf('  θ:   %.1f rad    dθ:  %.1f rad/s\n', thresholds(5), thresholds(6));
    fprintf('\n(Adjust thresholds in code if needed)\n');
    
    iteration = 0;
    while multiplier <= max_multiplier
        iteration = iteration + 1;
        noise_data.state_noise_amp = base_noise(1) * multiplier;
        noise_data.output_noise_amp = base_noise(2) * multiplier;
        
        [~, ~, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, initial_state);
        
        div_data.multipliers(end+1) = multiplier;
        div_data.rmse_values(end+1, :) = errors.rmse_states;
        
        % Check EACH STATE individually for divergence
        any_state_diverged_this_iteration = false;
        
        for state_idx = 1:6
            current_rmse = errors.rmse_states(state_idx);
            threshold = thresholds(state_idx);
            
            % Check if this state has exceeded its threshold
            if current_rmse > threshold
                % Mark as diverged if not already
                if ~div_data.(['actually_diverged_' state_names{state_idx}])
                    div_data.(['actually_diverged_' state_names{state_idx}]) = true;
                    div_data.(['div_point_' state_names{state_idx}]) = multiplier;
                    fprintf('  ✓ %s diverged at %.1fx (RMSE: %.4f > %.4f)\n', ...
                            display_names{state_idx}, multiplier, current_rmse, threshold);
                end
                any_state_diverged_this_iteration = true;
            end
        end
        
        % Update overall divergence flag
        if any_state_diverged_this_iteration && ~div_data.diverged
            div_data.diverged = true;
            div_data.divergence_point = multiplier;
            div_data.divergence_noise = [noise_data.state_noise_amp, noise_data.output_noise_amp];
        end
        
        % Progress indicator
        if mod(iteration, 10) == 0
            fprintf('  Progress: %.1f/%.1f (%.0f%%)\n', ...
                    multiplier, max_multiplier, (multiplier/max_multiplier)*100);
        end
        
        % Early exit if ALL states have diverged
        all_diverged = true;
        for state_idx = 1:6
            if ~div_data.(['actually_diverged_' state_names{state_idx}])
                all_diverged = false;
                break;
            end
        end
        
        if all_diverged
            fprintf('  All 6 states have diverged, stopping analysis.\n');
            break;
        end
        
        % Increment multiplier
        multiplier = multiplier + 0.2;
    end
    
    % Ensure ALL states have a div_point (use max tested for stable states)
    max_tested = max(div_data.multipliers);
    for state_idx = 1:6
        % If state never diverged, set div_point to max tested
        if ~div_data.(['actually_diverged_' state_names{state_idx}])
            div_data.(['div_point_' state_names{state_idx}]) = max_tested;
            fprintf('  ○ %s stable (max tested: %.1fx)\n', ...
                    display_names{state_idx}, max_tested);
        end
    end
    
    % Final summary
    fprintf('\n--- Individual State Divergence Results ---\n');
    diverged_count = 0;
    stable_count = 0;
    
    for state_idx = 1:6
        div_point = div_data.(['div_point_' state_names{state_idx}]);
        actually_diverged = div_data.(['actually_diverged_' state_names{state_idx}]);
        
        if actually_diverged
            diverged_count = diverged_count + 1;
            fprintf('  %-4s: DIVERGED at %6.1fx\n', ...
                    display_names{state_idx}, div_point);
        else
            stable_count = stable_count + 1;
            fprintf('  %-4s: STABLE   up to %6.1fx\n', ...
                    display_names{state_idx}, div_point);
        end
    end
    
    fprintf('\nSummary: %d diverged, %d stable\n', diverged_count, stable_count);
    fprintf('------------------------------------------\n');
    
    % Store summary stats
    div_data.diverged_count = diverged_count;
    div_data.stable_count = stable_count;
    div_data.max_multiplier_tested = max_tested;
end