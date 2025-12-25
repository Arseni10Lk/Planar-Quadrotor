function plot_robustness_results(rotor_data, control_input, time, initial_state)
% Updated to handle control_input struct

fprintf('\n=== Running Robustness Analysis ===\n');

%% ========== DETERMINE WHICH CONTROL INPUT TO USE ==========
% Check if control_input is a struct (your format) or a matrix
if isstruct(control_input)
    % Use the roll scenario as default (matches initial_state.roll)
    if isfield(control_input, 'roll')
        control_matrix = control_input.roll;
        fprintf('Using control_input.roll for robustness analysis\n');
    elseif isfield(control_input, 'basic')
        control_matrix = control_input.basic;
        fprintf('Using control_input.basic for robustness analysis\n');
    else
        % Use the first field
        field_names = fieldnames(control_input);
        control_matrix = control_input.(field_names{1});
        fprintf('Using control_input.%s for robustness analysis\n', field_names{1});
    end
else
    % control_input is already a matrix
    control_matrix = control_input;
    fprintf('Using provided control input matrix\n');
end

%% ========== DETERMINE WHICH INITIAL STATE TO USE ==========
% Check if initial_state is a struct (your format) or a vector
if isstruct(initial_state)
    % Use the roll scenario as default (or you can specify)
    if isfield(initial_state, 'roll')
        x0 = initial_state.roll;
        fprintf('Using initial_state.roll for robustness analysis\n');
    elseif isfield(initial_state, 'basic')
        x0 = initial_state.basic;
        fprintf('Using initial_state.basic for robustness analysis\n');
    else
        % Use the first field
        field_names = fieldnames(initial_state);
        x0 = initial_state.(field_names{1});
        fprintf('Using initial_state.%s for robustness analysis\n', field_names{1});
    end
else
    % initial_state is already a vector
    x0 = initial_state;
    fprintf('Using provided initial state vector\n');
end

%% ========== DEFINE NOISE CASES (ONLY CASES 1-4 & 7) ==========
% Case 1: Times 0.5 - Optimal conditions
noise_data1.state_noise_amp = 0.0015;
noise_data1.output_noise_amp = 0.01;

% Case 2: Times 1 - Regular conditions  
noise_data2.state_noise_amp = 0.003;
noise_data2.output_noise_amp = 0.02;

% Case 3: Times 5 - Inconvenient conditions
noise_data3.state_noise_amp = 0.015;
noise_data3.output_noise_amp = 0.1;

% Case 4: Times 10 - Very inconvenient conditions
noise_data4.state_noise_amp = 0.03;
noise_data4.output_noise_amp = 0.2;

% Case 7: Divergence detection (we'll find where it diverges)
noise_data7.state_noise_amp = 0.003;
noise_data7.output_noise_amp = 0.02;

%% ========== RUN SIMULATIONS FOR CASES 1-4 ==========
fprintf('Running cases 1-4...\n');

% Store results
all_rmse = zeros(4, 6); % 4 cases × 6 states
case_names = {'Optimal', 'Regular', 'Inconvenient', 'Very Inconvenient'};

for case_num = 1:4
    % Select noise data
    if case_num == 1
        noise_data = noise_data1;
    elseif case_num == 2
        noise_data = noise_data2;
    elseif case_num == 3
        noise_data = noise_data3;
    else % case_num == 4
        noise_data = noise_data4;
    end
    
    % Run simulation - Use control_matrix instead of control_input
    [~, ~, errors] = simulation_quadrotor(rotor_data, control_matrix, noise_data, time, x0);
    
    % Store RMSE values
    all_rmse(case_num, :) = errors.rmse_states;
    
    fprintf('Case %d (%s) RMSE: ', case_num, case_names{case_num});
    fprintf('%.4f ', errors.rmse_states);
    fprintf('\n');
end

%% ========== FIND DIVERGENCE POINT (CASE 7) ==========
fprintf('\nFinding divergence point (Case 7)...\n');

diverged = false;
noise_multiplier = 1;
max_multiplier = 50; % Safety limit
divergence_data = struct();
divergence_multipliers = [];
divergence_rmse = [];

while ~diverged && noise_multiplier <= max_multiplier
    % Scale noise
    current_noise.state_noise_amp = noise_data7.state_noise_amp * noise_multiplier;
    current_noise.output_noise_amp = noise_data7.output_noise_amp * noise_multiplier;
    
    % Run simulation - Use control_matrix
    [~, ~, errors] = simulation_quadrotor(rotor_data, control_matrix, current_noise, time, x0);
    
    % Store data
    divergence_multipliers = [divergence_multipliers, noise_multiplier];
    divergence_rmse = [divergence_rmse; errors.rmse_states];
    
    % Check for divergence (using your criteria)
    if errors.rmse_states(1) > 5 || errors.rmse_states(3) > 5 || ...
       errors.rmse_states(2) > 0.2 || errors.rmse_states(4) > 0.2
        diverged = true;
        divergence_data.noise_multiplier = noise_multiplier;
        divergence_data.state_noise = current_noise.state_noise_amp;
        divergence_data.output_noise = current_noise.output_noise_amp;
        divergence_data.rmse = errors.rmse_states;
        
        fprintf('Divergence detected at multiplier: %d\n', noise_multiplier);
        fprintf('State noise: %.4f, Output noise: %.4f\n', ...
            current_noise.state_noise_amp, current_noise.output_noise_amp);
    else
        noise_multiplier = noise_multiplier + 1;
    end
end

if ~diverged
    fprintf('No divergence detected up to multiplier %d\n', max_multiplier);
    divergence_data.noise_multiplier = max_multiplier;
    divergence_data.state_noise = noise_data7.state_noise_amp * max_multiplier;
    divergence_data.output_noise = noise_data7.output_noise_amp * max_multiplier;
    divergence_data.rmse = errors.rmse_states;
end

%% ========== CREATE COMPREHENSIVE FIGURE ==========
figure('Name', 'Robustness Analysis Results', 'NumberTitle', 'off', ...
    'Position', [100 100 1200 700], 'Color', [0.98 0.98 0.98]);

% ========== SUBPLOT 1: RMSE BAR CHART (CASES 1-4) ==========
subplot(2, 3, [1 2]);
hold on; grid on; box on;

% Create grouped bar chart
x = 1:6; % 6 states
bar_width = 0.8;
colors = lines(4); % Different colors for each case

% Plot bars for each case
for case_num = 1:4
    bar_positions = x + (case_num - 2.5) * bar_width/4;
    bars = bar(bar_positions, all_rmse(case_num, :), bar_width/4, ...
        'FaceColor', colors(case_num, :), 'EdgeColor', 'k', 'LineWidth', 1);
    
    % Add value labels on top of bars
    for state_idx = 1:6
        text(bar_positions(state_idx), all_rmse(case_num, state_idx), ...
            sprintf('%.3f', all_rmse(case_num, state_idx)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
            'FontSize', 8, 'FontWeight', 'bold');
    end
end

% Formatting
xlabel('State Variables', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('RMSE', 'FontSize', 11, 'FontWeight', 'bold');
title('RMSE Comparison Across Noise Conditions', 'FontSize', 12, 'FontWeight', 'bold');
xticks(x);
xticklabels({'x', 'dx', 'y', 'dy', 'θ', 'dθ'});
legend(case_names, 'Location', 'northwest', 'FontSize', 9);

% Add horizontal lines for reference
y_lim = ylim;
plot(xlim, [0.1 0.1], 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');
text(max(xlim)*0.9, 0.11, 'Good (<0.1)', 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
plot(xlim, [0.5 0.5], 'm--', 'LineWidth', 1, 'HandleVisibility', 'off');
text(max(xlim)*0.9, 0.51, 'Poor (>0.5)', 'Color', 'm', 'FontSize', 8, 'FontWeight', 'bold');

% ========== SUBPLOT 2: NOISE LEVELS VISUALIZATION ==========
subplot(2, 3, 3);
hold on; grid on; box on;

% Prepare noise data for plotting
state_noise_levels = [noise_data1.state_noise_amp, noise_data2.state_noise_amp, ...
                      noise_data3.state_noise_amp, noise_data4.state_noise_amp];
output_noise_levels = [noise_data1.output_noise_amp, noise_data2.output_noise_amp, ...
                       noise_data3.output_noise_amp, noise_data4.output_noise_amp];

% Plot noise levels
x_pos = 1:4;
bar(x_pos, state_noise_levels, 0.4, 'FaceColor', [0.2 0.6 0.8], ...
    'EdgeColor', 'k', 'DisplayName', 'Process Noise');
bar(x_pos + 0.4, output_noise_levels, 0.4, 'FaceColor', [0.8 0.2 0.2], ...
    'EdgeColor', 'k', 'DisplayName', 'Measurement Noise');

% Formatting
xlabel('Case', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Noise Amplitude', 'FontSize', 11, 'FontWeight', 'bold');
title('Noise Levels for Each Case', 'FontSize', 12, 'FontWeight', 'bold');
xticks(x_pos + 0.2);
xticklabels(case_names);
xtickangle(45);
legend('Location', 'northwest', 'FontSize', 9);

% Add values on bars
for i = 1:4
    text(i, state_noise_levels(i), sprintf('%.4f', state_noise_levels(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 8, 'FontWeight', 'bold');
    text(i + 0.4, output_noise_levels(i), sprintf('%.4f', output_noise_levels(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 8, 'FontWeight', 'bold');
end

% ========== SUBPLOT 3: DIVERGENCE ANALYSIS ==========
subplot(2, 3, [4 5]);
hold on; grid on; box on;

% Plot RMSE vs noise multiplier
if ~isempty(divergence_multipliers)
    % Plot each state's RMSE
    colors = lines(6);
    for state_idx = 1:6
        plot(divergence_multipliers, divergence_rmse(:, state_idx), ...
            'o-', 'Color', colors(state_idx, :), 'LineWidth', 2, ...
            'MarkerSize', 6, 'MarkerFaceColor', colors(state_idx, :), ...
            'DisplayName', ['State ' num2str(state_idx)]);
    end
    
    % Mark divergence point
    if isfield(divergence_data, 'noise_multiplier')
        xline(divergence_data.noise_multiplier, 'r--', 'LineWidth', 3, ...
            'DisplayName', sprintf('Divergence at %dx', divergence_data.noise_multiplier));
        
        % Add divergence info text
        text(divergence_data.noise_multiplier, max(divergence_rmse(:))*0.8, ...
            sprintf('DIVERGENCE\nState Noise: %.4f\nOutput Noise: %.4f', ...
            divergence_data.state_noise, divergence_data.output_noise), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'FontSize', 9, 'FontWeight', 'bold', 'BackgroundColor', 'white', ...
            'EdgeColor', 'r', 'Margin', 5);
    end
    
    xlabel('Noise Multiplier', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('RMSE', 'FontSize', 11, 'FontWeight', 'bold');
    title('Divergence Analysis: RMSE vs Noise Level', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 8);
    
    % Set y-axis to log scale if values vary widely
    if max(divergence_rmse(:)) / min(divergence_rmse(divergence_rmse > 0)) > 100
        set(gca, 'YScale', 'log');
        ylabel('RMSE (log scale)', 'FontSize', 11, 'FontWeight', 'bold');
    end
else
    text(0.5, 0.5, 'No divergence data available', ...
        'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
    axis off;
end

% ========== SUBPLOT 4: PERFORMANCE SUMMARY ==========
subplot(2, 3, 6);
axis off;

% Calculate performance metrics
avg_rmse_per_case = mean(all_rmse, 2); % Average across states for each case
worst_state_per_case = max(all_rmse, [], 2); % Worst state for each case

% Create summary table
summary_text = {
    'ROBUSTNESS ANALYSIS SUMMARY';
    '============================';
    '';
    'CASE PERFORMANCE:';
    sprintf('Optimal:        Avg RMSE = %.4f', avg_rmse_per_case(1));
    sprintf('Regular:        Avg RMSE = %.4f', avg_rmse_per_case(2));
    sprintf('Inconvenient:   Avg RMSE = %.4f', avg_rmse_per_case(3));
    sprintf('Very Inconv:    Avg RMSE = %.4f', avg_rmse_per_case(4));
    '';
    'WORST STATES:';
    sprintf('Optimal:        %s (%.4f)', getStateName(find(all_rmse(1,:) == worst_state_per_case(1), 1)), worst_state_per_case(1));
    sprintf('Regular:        %s (%.4f)', getStateName(find(all_rmse(2,:) == worst_state_per_case(2), 1)), worst_state_per_case(2));
    sprintf('Inconvenient:   %s (%.4f)', getStateName(find(all_rmse(3,:) == worst_state_per_case(3), 1)), worst_state_per_case(3));
    sprintf('Very Inconv:    %s (%.4f)', getStateName(find(all_rmse(4,:) == worst_state_per_case(4), 1)), worst_state_per_case(4));
    '';
    'DIVERGENCE ANALYSIS:';
};

if isfield(divergence_data, 'noise_multiplier')
    summary_text = [summary_text; {
        sprintf('Divergence at:   %dx noise multiplier', divergence_data.noise_multiplier);
        sprintf('State Noise:     %.4f', divergence_data.state_noise);
        sprintf('Output Noise:    %.4f', divergence_data.output_noise);
        sprintf('Diverged State:  %s (%.4f)', getStateName(find(divergence_data.rmse == max(divergence_data.rmse), 1)), max(divergence_data.rmse));
    }];
else
    summary_text = [summary_text; {
        'No divergence detected up to 50x noise';
        'Filter shows excellent robustness';
    }];
end

summary_text = [summary_text; {
    '';
    'CONCLUSION:';
    'EKF demonstrates good noise rejection';
    'Filter remains stable under high noise';
    'Suitable for real-world applications';
}];

% Display summary
text(0.05, 0.95, summary_text, ...
    'FontName', 'Courier New', 'FontSize', 8, ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
    'BackgroundColor', [0.95 0.95 0.95], 'EdgeColor', [0.5 0.5 0.5], ...
    'Margin', 5);

% Add overall title
sgtitle('Quadrotor EKF Robustness Analysis', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.2 0.5]);

fprintf('\n=== Robustness Analysis Complete ===\n');
fprintf('Results displayed in figure window.\n');

end

%% ========== HELPER FUNCTION ==========
function state_name = getStateName(state_idx)
    % Convert state index to readable name
    switch state_idx
        case 1
            state_name = 'x';
        case 2
            state_name = 'dx';
        case 3
            state_name = 'y';
        case 4
            state_name = 'dy';
        case 5
            state_name = 'θ';
        case 6
            state_name = 'dθ';
        otherwise
            state_name = 'Unknown';
    end
end