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

% Initialize noise_data structure
noise_data = struct();

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
divergence_data = find_divergence(rotor_data, control_input, time, initial_state);

fprintf('\n=== Analysis Complete === \n');
end

%% ================= HELPER FUNCTION =================
function div_data = find_divergence(rotor_data, control_input, time, initial_state)
    base_noise = [0.003, 0.02]; % Base noise levels
    max_multiplier = 50;
    multiplier = 0.2; % Start from 0.2x as suggested
    
    div_data.multipliers = [];
    div_data.rmse_values = [];
    div_data.diverged = false;
    
    % Define state names for field creation
    state_names = {'x', 'dx', 'y', 'dy', 'theta', 'dtheta'};
    div_data.state_names = state_names; % Add this field!
    
    % Initialize divergence points
    for i = 1:6
        div_data.(['div_point_' state_names{i}]) = 0;
    end

    while multiplier <= max_multiplier
        noise_data.state_noise_amp = base_noise(1) * multiplier;
        noise_data.output_noise_amp = base_noise(2) * multiplier;
        
        [~, ~, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, initial_state);
        
        div_data.multipliers(end+1) = multiplier;
        div_data.rmse_values(end+1, :) = errors.rmse_states;
        
        % Divergence criteria - check each state
        for state_idx = 1:6
            if errors.rmse_states(state_idx) > [5, 0.2, 5, 0.2, 0.5, 0.3](state_idx)
                if div_data.(['div_point_' state_names{state_idx}]) == 0
                    div_data.(['div_point_' state_names{state_idx}]) = multiplier;
                    fprintf('State %s diverged at %.1fx noise\n', state_names{state_idx}, multiplier);
                end
                div_data.diverged = true;
            end
        end
        
        % If divergence detected, store the point
        if div_data.diverged && ~isfield(div_data, 'divergence_point')
            div_data.divergence_point = multiplier;
            div_data.divergence_noise = [noise_data.state_noise_amp, noise_data.output_noise_amp];
            break;
        end
        
        multiplier = multiplier + 0.2; % Increment by 0.2
    end
    
    if ~div_data.diverged
        div_data.divergence_point = max_multiplier;
        div_data.divergence_noise = base_noise * max_multiplier;
        fprintf('No divergence up to %dx noise\n', max_multiplier);
    end
end