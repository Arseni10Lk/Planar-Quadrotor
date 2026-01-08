function plot_robustness_results(rmse_matrix, noise_matrix, divergence_data)
% PLOT_ROBUSTNESS_RESULTS - Visualize robustness analysis

% Force light theme
set(0, 'DefaultFigureColor', 'white');

% Get screen size
screen_size = get(0, 'ScreenSize');
fig_width = min(1000, screen_size(3) * 0.8);
fig_height = min(700, screen_size(4) * 0.8);
fig = figure('Name', 'Robustness Analysis', ...
             'NumberTitle', 'off', ...
             'Position', [(screen_size(3)-fig_width)/2, (screen_size(4)-fig_height)/2, fig_width, fig_height], ...
             'Color', 'white');

%% 1. RMSE Bar Chart - Fixed legend placement
ax1 = subplot(2, 3, 1);
bar(ax1, rmse_matrix', 'grouped');
xlabel(ax1, 'State', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax1, 'RMSE', 'FontSize', 10, 'FontWeight', 'bold');
title(ax1, 'RMSE Across Noise Conditions', 'FontSize', 11, 'FontWeight', 'bold');
set(ax1, 'XTickLabel', {'x', 'dx', 'y', 'dy', 'θ', 'dθ'});
grid(ax1, 'on');

% Move legend outside to avoid overlap
legend(ax1, {'Optimal', 'Regular', 'High', 'Very High'}, ...
       'Location', 'northoutside', ...
       'Orientation', 'horizontal', ...
       'FontSize', 9);

%% 2. Noise Levels - Fixed legend placement
ax2 = subplot(2, 3, 2);
bar(ax2, noise_matrix);
xlabel(ax2, 'Case', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax2, 'Noise Amplitude', 'FontSize', 10, 'FontWeight', 'bold');
title(ax2, 'Noise Levels', 'FontSize', 11, 'FontWeight', 'bold');
set(ax2, 'XTickLabel', {'Optimal', 'Regular', 'High', 'Very High'});
grid(ax2, 'on');

% Move legend outside
legend(ax2, {'Process', 'Measurement'}, ...
       'Location', 'northoutside', ...
       'Orientation', 'horizontal', ...
       'FontSize', 9);

%% 3. Divergence Analysis - Show ALL 6 states
ax3 = subplot(2, 3, [3 6]);  
hold(ax3, 'on'); 
grid(ax3, 'on'); 
box(ax3, 'on');

if ~isempty(divergence_data.multipliers) && ~isempty(divergence_data.rmse_values)
    % Define colors and markers for all 6 states
    colors = {'b', 'r', 'g', 'm', 'c', [0.5 0.3 0.1]};
    markers = {'o', 's', '^', 'v', 'd', 'p'};
    
    % Use state_names from divergence_data or default
    if isfield(divergence_data, 'state_names')
        state_names = divergence_data.state_names;
    else
        state_names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    end
    
    % Plot all 6 states
    for state_idx = 1:6
        plot(ax3, divergence_data.multipliers, divergence_data.rmse_values(:, state_idx), ...
             [colors{state_idx} '-' markers{state_idx}], ...
             'LineWidth', 1.5, ...
             'MarkerSize', 4, ...
             'MarkerFaceColor', colors{state_idx}, ...
             'DisplayName', state_names{state_idx});
    end
    
    % Add divergence lines for each state if they exist
    if isfield(divergence_data, 'diverged') && divergence_data.diverged
        % Check each state for divergence point
        for state_idx = 1:6
            field_name = ['div_point_' state_names{state_idx}];
            if isfield(divergence_data, field_name)
                div_point = divergence_data.(field_name);
                if div_point > 0
                    xline(ax3, div_point, '--', 'Color', colors{state_idx}, ...
                          'LineWidth', 1.2, 'Alpha', 0.6, ...
                          'DisplayName', sprintf('%s div (%.1fx)', state_names{state_idx}, div_point));
                end
            end
        end
        
        % Add overall divergence line
        if isfield(divergence_data, 'divergence_point') && divergence_data.divergence_point > 0
            xline(ax3, divergence_data.divergence_point, 'k--', 'LineWidth', 2, ...
                  'DisplayName', sprintf('Overall Divergence (%.1fx)', divergence_data.divergence_point));
        end
    end
    
    xlabel(ax3, 'Noise Multiplier', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel(ax3, 'RMSE', 'FontSize', 10, 'FontWeight', 'bold');
    title(ax3, 'RMSE Growth with Noise (All 6 States)', 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax3, 'Location', 'best', 'FontSize', 8, 'NumColumns', 2);
    
    % Add text if no divergence
    if isfield(divergence_data, 'diverged') && ~divergence_data.diverged
        text(ax3, 0.5, 0.95, 'No divergence detected', ...
             'Units', 'normalized', ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'top', ...
             'BackgroundColor', [0.9 0.9 0.9], ...
             'FontSize', 9, 'FontWeight', 'bold');
    end
    
else
    text(ax3, 0.5, 0.5, 'No divergence data', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    axis(ax3, 'off');
end

%% 4. Summary
ax4 = subplot(2, 3, [4 5]);
axis(ax4, 'off');

summary = {'SUMMARY:', '=========', ''};

% Add RMSE summary
summary{end+1} = 'RMSE PERFORMANCE:';
condition_names = {'Optimal', 'Regular', 'High', 'Very High'};
for i = 1:min(4, size(rmse_matrix, 1))
    avg_rmse = mean(rmse_matrix(i,:));
    summary{end+1} = sprintf('  %-12s: %.4f', condition_names{i}, avg_rmse);
end

summary{end+1} = '';
summary{end+1} = 'DIVERGENCE ANALYSIS:';

if isfield(divergence_data, 'diverged') && divergence_data.diverged
    if isfield(divergence_data, 'divergence_point')
        summary{end+1} = sprintf('  Divergence at %.1fx normal noise', divergence_data.divergence_point);
    end
    if isfield(divergence_data, 'divergence_noise')
        summary{end+1} = sprintf('  State noise: %.4f', divergence_data.divergence_noise(1));
        summary{end+1} = sprintf('  Output noise: %.4f', divergence_data.divergence_noise(2));
    end
    
    % List diverged states
    diverged_states = {};
    state_check_names = {'x', 'dx', 'y', 'dy', 'theta', 'dtheta'};
    for i = 1:6
        field_name = ['div_point_' state_check_names{i}];
        if isfield(divergence_data, field_name) && divergence_data.(field_name) > 0
            diverged_states{end+1} = state_check_names{i};
        end
    end
    
    if ~isempty(diverged_states)
        summary{end+1} = sprintf('  Diverged states: %s', strjoin(diverged_states, ', '));
    end
    
else
    summary{end+1} = '  No divergence detected';
    if isfield(divergence_data, 'multipliers') && ~isempty(divergence_data.multipliers)
        summary{end+1} = sprintf('  Up to %.1fx noise tested', max(divergence_data.multipliers));
    end
end

text(ax4, 0.05, 0.95, summary, ...
     'FontName', 'FixedWidth', ...
     'FontSize', 9, ...
     'VerticalAlignment', 'top', ...
     'BackgroundColor', [0.95 0.95 0.95]);

%% Overall title
sgtitle('EKF Robustness Analysis', 'FontSize', 14, 'FontWeight', 'bold');

drawnow;
end