function [rmse_matrix, noise_matrix, divergence_data] = robustness(rotor_data, control_input, time, initial_state)
% ROBUSTNESS - Run robustness tests and return data for plotting
% Returns: rmse_matrix (4x6), noise_matrix (4x2), divergence_data

fprintf('\n=== Running Robustness Analysis ===\n');

% Define 4 noise cases (Cases 1-4)
noise_cases = [
    0.0015, 0.01;  % Case 1: Optimal
    0.003,  0.02;  % Case 2: Regular
    0.015,  0.1;   % Case 3: High noise
    0.03,   0.2    % Case 4: Very high noise
];

num_cases = size(noise_cases, 1);
rmse_matrix = zeros(num_cases, 6);
noise_matrix = noise_cases;

% Run simulations for Cases 1–4
for case_num = 1:num_cases
    noise_data.state_noise_amp = noise_cases(case_num, 1);
    noise_data.output_noise_amp = noise_cases(case_num, 2);
    
    [~, ~, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, initial_state);
    rmse_matrix(case_num, :) = errors.rmse_states;
    
    fprintf('Case %d RMSE: ', case_num);
    fprintf('%.4f ', errors.rmse_states);
    fprintf('\n');
end

% Run divergence analysis (Case 7)
divergence_data = find_divergence(rotor_data, control_input, time, initial_state);

fprintf('\n=== Analysis Complete ===\n');
end

% --- Local helper function ---
function div_data = find_divergence(rotor_data, control_input, time, initial_state)
    base_noise = [0.003, 0.02];  % Base noise levels
    max_multiplier = 50;
    multiplier = 1;
    
    div_data.multipliers = [];
    div_data.rmse_values = [];
    div_data.diverged = false;
    
    while multiplier <= max_multiplier
        noise_data.state_noise_amp = base_noise(1) * multiplier;
        noise_data.output_noise_amp = base_noise(2) * multiplier;
        
        [~, ~, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, initial_state);
        
        div_data.multipliers(end+1) = multiplier;
        div_data.rmse_values(end+1, :) = errors.rmse_states;
        
        % Divergence criteria
        if errors.rmse_states(1) > 5 || errors.rmse_states(3) > 5 || ...
           errors.rmse_states(2) > 0.2 || errors.rmse_states(4) > 0.2
            div_data.diverged = true;
            div_data.divergence_point = multiplier;
            div_data.divergence_noise = [noise_data.state_noise_amp, noise_data.output_noise_amp];
            fprintf('Divergence at %dx noise multiplier\n', multiplier);
            break;
        end
        
        multiplier = multiplier + 1;
    end
    
    if ~div_data.diverged
        div_data.divergence_point = max_multiplier;
        div_data.divergence_noise = base_noise * max_multiplier;
        fprintf('No divergence up to %dx noise\n', max_multiplier);
    end
end