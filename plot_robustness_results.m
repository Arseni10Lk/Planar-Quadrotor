function plot_robustness_results(rmse_matrix, noise_matrix, divergence_data)
% PLOT_ROBUSTNESS_RESULTS - Visualize robustness analysis with individual state divergences

% Force light theme
set(0, 'DefaultFigureColor', 'white');

% Get screen size
screen_size = get(0, 'ScreenSize');
fig_width = min(1400, screen_size(3) * 0.9);
fig_height = min(900, screen_size(4) * 0.85);
fig_x = (screen_size(3) - fig_width) / 2;
fig_y = (screen_size(4) - fig_height) / 2;

% Create figure
fig = figure('Name', 'EKF Robustness Analysis: Individual State Divergence', ...
             'NumberTitle', 'off', ...
             'Position', [fig_x, fig_y, fig_width, fig_height], ...
             'Color', 'white');

%% ================= 1. RMSE BAR CHART =================
ax1 = subplot(3, 4, [1 2]);
bar(ax1, rmse_matrix', 'grouped');
xlabel(ax1, 'State', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax1, 'RMSE', 'FontSize', 10, 'FontWeight', 'bold');
title(ax1, 'RMSE Across Noise Conditions', 'FontSize', 11, 'FontWeight', 'bold');
set(ax1, 'XTickLabel', {'x', 'dx', 'y', 'dy', 'θ', 'dθ'});
grid(ax1, 'on');
legend(ax1, {'Optimal', 'Regular', 'High', 'Very High'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);

%% ================= 2. NOISE LEVELS =================
ax2 = subplot(3, 4, [3 4]);
bar(ax2, noise_matrix);
xlabel(ax2, 'Case', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax2, 'Noise Amplitude', 'FontSize', 10, 'FontWeight', 'bold');
title(ax2, 'Noise Levels', 'FontSize', 11, 'FontWeight', 'bold');
set(ax2, 'XTickLabel', {'Optimal', 'Regular', 'High', 'Very High'});
grid(ax2, 'on');
legend(ax2, {'Process', 'Measurement'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);

%% ================= 3. RMSE GROWTH FOR X AND DX (LOGARITHMIC SCALE) =================
ax3 = subplot(3, 4, [5 6]);
hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
set(ax3, 'YScale', 'log');  % Set logarithmic scale for Y-axis

if ~isempty(divergence_data.multipliers) && ~isempty(divergence_data.rmse_values)
    % Colors for states
    colors = lines(6);
    state_names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    
    % Plot only x and dx states
    xdx_indices = [1, 2];  % x and dx states
    
    plot_handles_xdx = [];
    
    for plot_idx = 1:2
        state_idx = xdx_indices(plot_idx);
        data_state_name = divergence_data.state_names{state_idx};
        
        % Get divergence point
        div_field = ['div_point_' data_state_name];
        actual_div_field = ['actually_diverged_' data_state_name];
        
        if isfield(divergence_data, div_field) && isfield(divergence_data, actual_div_field)
            div_point = divergence_data.(div_field);
            actually_diverged = divergence_data.(actual_div_field);
            
            % Get all data
            x_all = divergence_data.multipliers;
            y_all = divergence_data.rmse_values(:, state_idx);
            
            % Ensure positive values for log scale
            y_all_pos = y_all;
            y_all_pos(y_all_pos <= 0) = 1e-10;  % Small positive value for log scale
            
            if actually_diverged
                % For diverged states: truncate at divergence point
                idx = find(x_all <= div_point);
                if ~isempty(idx)
                    x_plot = x_all(idx);
                    y_plot = y_all_pos(idx);
                    
                    % Plot the truncated curve
                    h = plot(ax3, x_plot, y_plot, '-', ...
                             'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                             'DisplayName', sprintf('%s (diverged)', state_names{state_idx}));
                    
                    % Add marker at divergence point
                    plot(ax3, div_point, y_plot(end), 'o', ...
                         'MarkerSize', 10, 'MarkerFaceColor', colors(state_idx,:), ...
                         'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
                     
                    % Add vertical line at divergence
                    xline(ax3, div_point, '--', 'Color', colors(state_idx,:), ...
                          'LineWidth', 1.8, 'Alpha', 0.7, 'HandleVisibility', 'off');
                     
                    % Add text label
                    y_range = ylim(ax3);
                    text_y = 10^(log10(y_range(2)) * 0.9 - (plot_idx-1) * 0.2);
                    text(ax3, div_point, text_y, sprintf('%s: %.1fx', state_names{state_idx}, div_point), ...
                         'Color', colors(state_idx,:), 'FontSize', 9, 'FontWeight', 'bold', ...
                         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                         'BackgroundColor', [1 1 1 0.9]);
                end
            else
                % For stable states: plot full curve
                h = plot(ax3, x_all, y_all_pos, '-', ...
                         'Color', colors(state_idx,:) * 0.7, 'LineWidth', 2.0, ...
                         'DisplayName', sprintf('%s (stable)', state_names{state_idx}));
                     
                % Add marker at the end
                last_x = x_all(end);
                last_y = y_all_pos(end);
                plot(ax3, last_x, last_y, 's', ...
                     'MarkerSize', 10, 'MarkerFaceColor', colors(state_idx,:) * 0.7, ...
                     'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
                 
                % Add vertical line at max tested
                xline(ax3, last_x, ':', 'Color', colors(state_idx,:) * 0.7, ...
                      'LineWidth', 1.8, 'Alpha', 0.7, 'HandleVisibility', 'off');
                 
                % Add text label
                y_range = ylim(ax3);
                text_y = 10^(log10(y_range(1)) * 1.1 + (plot_idx-1) * 0.2);
                text(ax3, last_x, text_y, sprintf('%s: %.1fx', state_names{state_idx}, last_x), ...
                     'Color', colors(state_idx,:) * 0.7, 'FontSize', 8, 'FontWeight', 'bold', ...
                     'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                     'BackgroundColor', [1 1 1 0.9]);
            end
            
            plot_handles_xdx = [plot_handles_xdx, h];
        end
    end
    
    xlabel(ax3, 'Noise Multiplier (x baseline)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel(ax3, 'RMSE (log scale)', 'FontSize', 10, 'FontWeight', 'bold');
    title(ax3, 'RMSE Growth: x and dx States (Log Scale)', 'FontSize', 11, 'FontWeight', 'bold');
    
    % Add legend
    legend(plot_handles_xdx, 'Location', 'best', 'FontSize', 9);
    
    % Adjust axis limits
    if ~isempty(x_all)
        xlim(ax3, [min(x_all) * 0.95, max(x_all) * 1.05]);
    end
    
    % Add grid lines for log scale
    grid(ax3, 'on');
    grid(ax3, 'minor');
    ax3.MinorGridAlpha = 0.2;
    ax3.GridAlpha = 0.3;
    
else
    text(ax3, 0.5, 0.5, 'No divergence data available', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    axis(ax3, 'off');
end

%% ================= 4. RMSE GROWTH FOR Y, DY, θ, Dθ (LINEAR SCALE) =================
ax4 = subplot(3, 4, [7 8]);
hold(ax4, 'on'); grid(ax4, 'on'); box(ax4, 'on');

if ~isempty(divergence_data.multipliers) && ~isempty(divergence_data.rmse_values)
    % Colors for states
    colors = lines(6);
    state_names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    
    % Plot y, dy, θ, and dθ states (states 3, 4, 5, 6)
    remaining_indices = [3, 4, 5, 6];
    
    plot_handles_remaining = [];
    
    for plot_idx = 1:4
        state_idx = remaining_indices(plot_idx);
        data_state_name = divergence_data.state_names{state_idx};
        
        % Get divergence point
        div_field = ['div_point_' data_state_name];
        actual_div_field = ['actually_diverged_' data_state_name];
        
        if isfield(divergence_data, div_field) && isfield(divergence_data, actual_div_field)
            div_point = divergence_data.(div_field);
            actually_diverged = divergence_data.(actual_div_field);
            
            % Get all data
            x_all = divergence_data.multipliers;
            y_all = divergence_data.rmse_values(:, state_idx);
            
            if actually_diverged
                % For diverged states: truncate at divergence point
                idx = find(x_all <= div_point);
                if ~isempty(idx)
                    x_plot = x_all(idx);
                    y_plot = y_all(idx);
                    
                    % Plot the truncated curve
                    h = plot(ax4, x_plot, y_plot, '-', ...
                             'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                             'DisplayName', sprintf('%s (diverged)', state_names{state_idx}));
                    
                    % Add marker at divergence point
                    plot(ax4, div_point, y_plot(end), 'o', ...
                         'MarkerSize', 10, 'MarkerFaceColor', colors(state_idx,:), ...
                         'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
                     
                    % Add vertical line at divergence
                    xline(ax4, div_point, '--', 'Color', colors(state_idx,:), ...
                          'LineWidth', 1.8, 'Alpha', 0.7, 'HandleVisibility', 'off');
                     
                    % Add text label
                    y_range = ylim(ax4);
                    text_y = y_range(2) * 0.9 - (plot_idx-1) * (y_range(2) * 0.15);
                    text(ax4, div_point + 0.5, text_y, sprintf('%s: %.1fx', state_names{state_idx}, div_point), ...
                         'Color', colors(state_idx,:), 'FontSize', 9, 'FontWeight', 'bold', ...
                         'HorizontalAlignment', 'left', 'BackgroundColor', [1 1 1 0.9]);
                end
            else
                % For stable states: plot full curve
                h = plot(ax4, x_all, y_all, '-', ...
                         'Color', colors(state_idx,:) * 0.7, 'LineWidth', 2.0, ...
                         'DisplayName', sprintf('%s (stable)', state_names{state_idx}));
                     
                % Add marker at the end
                last_x = x_all(end);
                last_y = y_all(end);
                plot(ax4, last_x, last_y, 's', ...
                     'MarkerSize', 10, 'MarkerFaceColor', colors(state_idx,:) * 0.7, ...
                     'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
                 
                % Add vertical line at max tested
                xline(ax4, last_x, ':', 'Color', colors(state_idx,:) * 0.7, ...
                      'LineWidth', 1.8, 'Alpha', 0.7, 'HandleVisibility', 'off');
                 
                % Add text label
                y_range = ylim(ax4);
                text_y = y_range(1) * 0.9 + (plot_idx-1) * (abs(y_range(1)) * 0.15);
                text(ax4, last_x + 0.5, text_y, sprintf('%s: %.1fx', state_names{state_idx}, last_x), ...
                     'Color', colors(state_idx,:) * 0.7, 'FontSize', 8, 'FontWeight', 'bold', ...
                     'HorizontalAlignment', 'left', 'BackgroundColor', [1 1 1 0.9]);
            end
            
            plot_handles_remaining = [plot_handles_remaining, h];
        end
    end
    
    xlabel(ax4, 'Noise Multiplier (x baseline)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel(ax4, 'RMSE', 'FontSize', 10, 'FontWeight', 'bold');
    title(ax4, 'RMSE Growth: y, dy, θ, and dθ States', 'FontSize', 11, 'FontWeight', 'bold');
    
    % Add legend
    legend(plot_handles_remaining, 'Location', 'best', 'FontSize', 9, 'NumColumns', 2);
    
    % Add explanation
    explanation = {'○ : Diverged', '□ : Stable (max tested)', '-- : Divergence point', ': : Max tested'};
    text(ax4, 0.02, 0.98, explanation, ...
         'Units', 'normalized', 'HorizontalAlignment', 'left', ...
         'VerticalAlignment', 'top', 'FontSize', 8, ...
         'BackgroundColor', [1 1 1 0.9], 'EdgeColor', [0.7 0.7 0.7]);
    
    % Adjust axis limits
    if ~isempty(x_all)
        xlim(ax4, [min(x_all) * 0.95, max(x_all) * 1.05]);
    end
    
else
    text(ax4, 0.5, 0.5, 'No divergence data available', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    axis(ax4, 'off');
end

%% ================= 5. DIVERGENCE SUMMARY TABLE =================
ax5 = subplot(3, 4, [9 12]);
axis(ax5, 'off');

% Prepare summary
summary = {'INDIVIDUAL STATE DIVERGENCE SUMMARY', ...
           '====================================', ...
           ''};

if isfield(divergence_data, 'state_names')
    state_display_names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    max_tested = max(divergence_data.multipliers);
    
    summary{end+1} = 'State    Status         Ended at (x)';
    summary{end+1} = '------   -----------    ------------';
    
    for state_idx = 1:6
        data_state_name = divergence_data.state_names{state_idx};
        div_field = ['div_point_' data_state_name];
        actual_div_field = ['actually_diverged_' data_state_name];
        
        if isfield(divergence_data, div_field) && isfield(divergence_data, actual_div_field)
            div_point = divergence_data.(div_field);
            actually_diverged = divergence_data.(actual_div_field);
            
            if actually_diverged
                status = 'DIVERGED';
                end_point = div_point;
                status_color = '[1.0 0.0 0.0]';
            else
                status = 'STABLE';
                end_point = max_tested;
                status_color = '[0.0 0.5 0.0]';
            end
            
            summary{end+1} = sprintf('  %-4s    %-11s    %8.1fx', ...
                                    state_display_names{state_idx}, status, end_point);
        end
    end
    
    summary{end+1} = '';
    summary{end+1} = sprintf('Testing Range: %.1f - %.1fx baseline noise', ...
                            min(divergence_data.multipliers), max_tested);
    
    if isfield(divergence_data, 'diverged_count') && isfield(divergence_data, 'stable_count')
        summary{end+1} = sprintf('Summary: %d diverged, %d stable', ...
                                divergence_data.diverged_count, divergence_data.stable_count);
    end
    
else
    summary{end+1} = 'No state divergence data available';
end

% Display summary
text(0.05, 0.95, summary, ...
     'FontName', 'FixedWidth', 'FontSize', 9.5, ...
     'VerticalAlignment', 'top', 'BackgroundColor', [0.97 0.97 0.97], ...
     'EdgeColor', [0.8 0.8 0.8], 'LineWidth', 1);

title(ax5, 'State-by-State Divergence Analysis', 'FontWeight', 'bold', 'FontSize', 11);

%% ================= FINAL TOUCHES =================
sgtitle({'Extended Kalman Filter: Robustness Analysis', ...
         'Split view: x/dx (log scale) vs. y/dy/θ/dθ (linear scale)'}, ...
        'FontSize', 14, 'FontWeight', 'bold');

annotation('textbox', [0.02, 0.02, 0.2, 0.03], ...
           'String', sprintf('Generated: %s', datestr(now)), ...
           'FontSize', 8, 'EdgeColor', 'none', 'HorizontalAlignment', 'left');

% Add annotation about log scale
annotation('textbox', [0.5, 0.25, 0.4, 0.05], ...
           'String', {'Note: x and dx plotted on logarithmic scale for better visibility', ...
                     'y, dy, θ, and dθ plotted on standard linear scale'}, ...
           'FontSize', 8, 'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
           'BackgroundColor', [1 1 1 0.8]);

drawnow;

% Console output
fprintf('\n=== INDIVIDUAL STATE DIVERGENCE FINAL REPORT ===\n');
if isfield(divergence_data, 'state_names')
    for state_idx = 1:6
        data_state_name = divergence_data.state_names{state_idx};
        display_name = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
        div_field = ['div_point_' data_state_name];
        actual_div_field = ['actually_diverged_' data_state_name];
        
        if isfield(divergence_data, div_field) && isfield(divergence_data, actual_div_field)
            div_point = divergence_data.(div_field);
            actually_diverged = divergence_data.(actual_div_field);
            
            if actually_diverged
                fprintf('  %s DIVERGED at: %.1fx noise\n', display_name{state_idx}, div_point);
            else
                fprintf('  %s STABLE up to: %.1fx noise\n', display_name{state_idx}, max(divergence_data.multipliers));
            end
        end
    end
end
fprintf('================================================\n');
fprintf('Plot layout:\n');
fprintf('  - Top: RMSE bar chart and noise levels\n');
fprintf('  - Middle-left: x and dx states (log scale)\n');
fprintf('  - Middle-right: y, dy, θ, dθ states (linear scale)\n');
fprintf('  - Bottom: Divergence summary table\n');
end