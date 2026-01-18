function plot_robustness_results(rmse_matrix, noise_matrix, divergence_data)
% PLOT_ROBUSTNESS_RESULTS - Visualize robustness analysis
% Updated: Linear graph (ax4) now prefers TOP placement unless obstructing data.

% Force light theme
set(0, 'DefaultFigureColor', 'white');

% Get screen size
screen_size = get(0, 'ScreenSize');
fig_width = min(1400, screen_size(3) * 0.9);
fig_height = min(900, screen_size(4) * 0.85);
fig_x = (screen_size(3) - fig_width) / 2;
fig_y = (screen_size(4) - fig_height) / 2;

% Create figure
fig = figure('Name', 'EKF vs Running Filter: Robustness Analysis', ...
             'NumberTitle', 'off', ...
             'Position', [fig_x, fig_y, fig_width, fig_height], ...
             'Color', 'white');

%% ================= 1. RMSE BAR CHART (EKF Only) =================
ax1 = subplot(3, 4, [1 2]);
bar(ax1, rmse_matrix', 'grouped');
xlabel(ax1, 'State', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax1, 'RMSE (EKF)', 'FontSize', 10, 'FontWeight', 'bold');
title(ax1, 'EKF RMSE Across Noise Conditions', 'FontSize', 11, 'FontWeight', 'bold');
set(ax1, 'XTickLabel', {'x', 'dx', 'y', 'dy', 'θ', 'dθ'});
grid(ax1, 'on');
legend(ax1, {'Optimal', 'Regular', 'High', 'Very High'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);

%% ================= 2. NOISE LEVELS =================
ax2 = subplot(3, 4, [3 4]);
bar(ax2, noise_matrix);
xlabel(ax2, 'Case', 'FontSize', 10, 'FontWeight', 'bold');
ylabel(ax2, 'Variance', 'FontSize', 10, 'FontWeight', 'bold');
title(ax2, 'Noise Levels', 'FontSize', 11, 'FontWeight', 'bold');
set(ax2, 'XTickLabel', {'Optimal', 'Regular', 'High', 'Very High'});
grid(ax2, 'on');
legend(ax2, {'Process', 'Measurement'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);

%% ================= 3. RMSE GROWTH FOR X AND DX (LOGARITHMIC SCALE) =================
ax3 = subplot(3, 4, [5 6]);
ax3_pos = get(ax3, 'Position'); 
hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
set(ax3, 'YScale', 'log');

labels_to_plot_3 = struct('x', {}, 'y_actual', {}, 'str', {}, 'color', {});

if ~isempty(divergence_data.multipliers) && ~isempty(divergence_data.rmse_values)
    colors = lines(6);
    state_names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    xdx_indices = [1, 2];
    plot_handles_xdx = [];
    
    for plot_idx = 1:2
        state_idx = xdx_indices(plot_idx);
        data_state_name = divergence_data.state_names{state_idx};
        
        % ---------------- EKF (SOLID) ----------------
        div_field = ['div_point_' data_state_name];
        actual_div_field = ['actually_diverged_' data_state_name];
        
        if isfield(divergence_data, div_field)
            div_point = divergence_data.(div_field);
            actually_diverged = divergence_data.(actual_div_field);
            x_all = divergence_data.multipliers;
            y_all = divergence_data.rmse_values(:, state_idx);
            y_all_pos = y_all; y_all_pos(y_all_pos <= 0) = 1e-10; 
            
            if actually_diverged
                idx = find(x_all <= div_point);
                if ~isempty(idx)
                    h = plot(ax3, x_all(idx), y_all_pos(idx), '-', ...
                             'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                             'DisplayName', sprintf('%s (EKF)', state_names{state_idx}));
                    
                    final_y = y_all_pos(idx(end));
                    plot(ax3, div_point, final_y, 'o', ...
                         'MarkerSize', 8, 'MarkerFaceColor', colors(state_idx,:), 'MarkerEdgeColor', 'k');
                    xline(ax3, div_point, '-', 'Color', colors(state_idx,:), 'LineWidth', 1.0, 'Alpha', 0.5);
                    
                    labels_to_plot_3(end+1) = struct('x', div_point, 'y_actual', final_y, ...
                        'str', sprintf('%.1fx', div_point), 'color', colors(state_idx,:));
                end
            else
                h = plot(ax3, x_all, y_all_pos, '-', 'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                         'DisplayName', sprintf('%s (EKF)', state_names{state_idx}));
                plot(ax3, x_all(end), y_all_pos(end), 's', ...
                     'MarkerSize', 8, 'MarkerFaceColor', colors(state_idx,:), 'MarkerEdgeColor', 'k');
            end
            plot_handles_xdx = [plot_handles_xdx, h];
        end

        % ---------------- RUNNING FILTER (DASHED) ----------------
        if isfield(divergence_data, 'rmse_values_running')
            div_field_run = ['div_point_running_' data_state_name];
            actual_div_field_run = ['actually_diverged_running_' data_state_name];
            y_all_run = divergence_data.rmse_values_running(:, state_idx);
            y_all_run_pos = y_all_run; y_all_run_pos(y_all_run_pos <= 0) = 1e-10;

            div_point_run = divergence_data.(div_field_run);
            actually_diverged_run = divergence_data.(actual_div_field_run);

            if actually_diverged_run
                idx = find(x_all <= div_point_run);
                if ~isempty(idx)
                    h_run = plot(ax3, x_all(idx), y_all_run_pos(idx), '--', ...
                             'Color', colors(state_idx,:), 'LineWidth', 1.5, ...
                             'DisplayName', sprintf('%s (Running Mean)', state_names{state_idx}));
                    
                    final_y_run = y_all_run_pos(idx(end));
                    plot(ax3, div_point_run, final_y_run, 'x', ...
                         'MarkerSize', 8, 'Color', colors(state_idx,:), 'LineWidth', 1.5);
                    xline(ax3, div_point_run, '--', 'Color', colors(state_idx,:), 'LineWidth', 0.8, 'Alpha', 0.4);
                    
                    labels_to_plot_3(end+1) = struct('x', div_point_run, 'y_actual', final_y_run, ...
                        'str', sprintf('%.1fx', div_point_run), 'color', colors(state_idx,:)*0.8);
                end
            else
                 h_run = plot(ax3, x_all, y_all_run_pos, '--', ...
                             'Color', colors(state_idx,:), 'LineWidth', 1.5, ...
                             'DisplayName', sprintf('%s (Running Mean)', state_names{state_idx}));
            end
            if length(plot_handles_xdx) < 4, plot_handles_xdx = [plot_handles_xdx, h_run]; end
        end
    end
    
    % ---- STACKING LOGIC FOR AX3 (LOG SCALE) ----
    % Stacks from Bottom-Up (Skyline)
    if ~isempty(labels_to_plot_3)
        [~, sort_idx] = sort([labels_to_plot_3.x]);
        labels_to_plot_3 = labels_to_plot_3(sort_idx);
        
        y_lims = ylim(ax3);
        y_min_log = log10(y_lims(1)); y_max_log = log10(y_lims(2)); y_range_log = y_max_log - y_min_log;
        x_lims = xlim(ax3); if isempty(x_lims), x_lims = [0 1]; end
        x_range = x_lims(2) - x_lims(1);
        MIN_X_DIST = x_range * 0.22;
        
        placed_labels = struct('x', {}, 'row', {}); 
        for k = 1:length(labels_to_plot_3)
            lbl = labels_to_plot_3(k);
            row = 0; placed = false;
            while ~placed
                collision = false;
                for p = 1:length(placed_labels)
                    if placed_labels(p).row == row && abs(lbl.x - placed_labels(p).x) < MIN_X_DIST
                        collision = true; break;
                    end
                end
                if ~collision
                    placed = true;
                    placed_labels(end+1) = struct('x', lbl.x, 'row', row); %#ok<AGROW>
                    pct_up = 0.05 + (row * 0.12);
                    y_pos = 10^(y_min_log + pct_up * y_range_log);
                    
                    plot(ax3, [lbl.x, lbl.x], [lbl.y_actual, y_pos], ':', 'Color', [lbl.color, 0.6], 'LineWidth', 1.0);
                    text(ax3, lbl.x, y_pos, lbl.str, 'Color', lbl.color, 'FontSize', 8, 'FontWeight', 'bold', ...
                         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'BackgroundColor', [1 1 1 1.0], 'EdgeColor', 'none', 'Margin', 2);
                else
                    row = row + 1;
                end
            end
        end
    end
    
    xlabel(ax3, 'Noise Multiplier (x baseline)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel(ax3, 'RMSE (log scale)', 'FontSize', 10, 'FontWeight', 'bold');
    title(ax3, 'RMSE Growth: x and dx (Log Scale)', 'FontSize', 11, 'FontWeight', 'bold');
    
    hL3 = legend(plot_handles_xdx, 'NumColumns', 1, 'FontSize', 8);
    set(hL3, 'Position', [ax3_pos(1), ax3_pos(2) - 0.23, 0.1, 0.14], 'Box', 'on');
    set(ax3, 'Position', ax3_pos);
    if ~isempty(x_all), xlim(ax3, [min(x_all) * 0.95, max(x_all) * 1.05]); end
    grid(ax3, 'on'); grid(ax3, 'minor'); ax3.MinorGridAlpha = 0.2; ax3.GridAlpha = 0.3;
else
    text(ax3, 0.5, 0.5, 'No divergence data', 'HorizontalAlignment', 'center'); axis(ax3, 'off');
end

%% ================= 4. RMSE GROWTH FOR Y, DY, θ, Dθ (LINEAR SCALE) =================
% Modified: PREFER TOP PLACEMENT (unless blocking data), THEN BOTTOM
ax4 = subplot(3, 4, [7 8]);
ax4_pos = get(ax4, 'Position');
hold(ax4, 'on'); grid(ax4, 'on'); box(ax4, 'on');

labels_to_plot_4 = struct('x', {}, 'y_actual', {}, 'str', {}, 'color', {});

if ~isempty(divergence_data.multipliers) && ~isempty(divergence_data.rmse_values)
    colors = lines(6);
    remaining_indices = [3, 4, 5, 6];
    plot_handles_remaining = [];
    
    for plot_idx = 1:4
        state_idx = remaining_indices(plot_idx);
        data_state_name = divergence_data.state_names{state_idx};
        x_all = divergence_data.multipliers;
        y_all = divergence_data.rmse_values(:, state_idx);
        
        % ---------------- EKF ----------------
        div_field = ['div_point_' data_state_name];
        actual_div_field = ['actually_diverged_' data_state_name];
        
        if isfield(divergence_data, div_field)
            div_point = divergence_data.(div_field);
            actually_diverged = divergence_data.(actual_div_field);
            
            if actually_diverged
                idx = find(x_all <= div_point);
                if ~isempty(idx)
                    h = plot(ax4, x_all(idx), y_all(idx), '-', 'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                             'DisplayName', sprintf('%s (EKF)', state_names{state_idx}));
                    
                    final_y = y_all(idx(end));
                    plot(ax4, div_point, final_y, 'o', 'MarkerSize', 6, 'MarkerFaceColor', colors(state_idx,:), 'MarkerEdgeColor', 'k');
                    xline(ax4, div_point, '-', 'Color', colors(state_idx,:), 'LineWidth', 1.0, 'Alpha', 0.4);
                    
                    % STORE LABEL
                    labels_to_plot_4(end+1) = struct('x', div_point, 'y_actual', final_y, ...
                        'str', sprintf('%.1fx', div_point), 'color', colors(state_idx,:));
                end
            else
                h = plot(ax4, x_all, y_all, '-', 'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                         'DisplayName', sprintf('%s (EKF)', state_names{state_idx}));
                xline(ax4, x_all(end), ':', 'Color', colors(state_idx,:), 'LineWidth', 1.0, 'Alpha', 0.4);
            end
            plot_handles_remaining = [plot_handles_remaining, h];
        end

        % ---------------- RUNNING ----------------
        if isfield(divergence_data, 'rmse_values_running') && state_idx ~= 3 && state_idx ~= 4
            div_field_run = ['div_point_running_' data_state_name];
            actual_div_field_run = ['actually_diverged_running_' data_state_name];
            y_all_run = divergence_data.rmse_values_running(:, state_idx);
            div_point_run = divergence_data.(div_field_run);
            actually_diverged_run = divergence_data.(actual_div_field_run);
            
            if actually_diverged_run
                idx = find(x_all <= div_point_run);
                if ~isempty(idx)
                    h_run = plot(ax4, x_all(idx), y_all_run(idx), '--', 'Color', colors(state_idx,:), 'LineWidth', 1.5, ...
                             'DisplayName', sprintf('%s (Running Mean)', state_names{state_idx}));
                    
                    final_y_run = y_all_run(idx(end));
                    plot(ax4, div_point_run, final_y_run, 'x', 'MarkerSize', 6, 'Color', colors(state_idx,:), 'LineWidth', 1.5);
                    xline(ax4, div_point_run, '--', 'Color', colors(state_idx,:), 'LineWidth', 0.8, 'Alpha', 0.3);
                    
                    % STORE LABEL
                    labels_to_plot_4(end+1) = struct('x', div_point_run, 'y_actual', final_y_run, ...
                        'str', sprintf('%.1fx', div_point_run), 'color', colors(state_idx,:)*0.8);
                end
            else
                h_run = plot(ax4, x_all, y_all_run, '--', 'Color', colors(state_idx,:), 'LineWidth', 1.5, ...
                             'DisplayName', sprintf('%s (Running Mean)', state_names{state_idx}));
            end
            if length(plot_handles_remaining) < 8
               plot_handles_remaining = [plot_handles_remaining, h_run];
            end
        end
    end
    
    % ---- NEW LOGIC: PREFER TOP, FALLBACK TO BOTTOM ----
    if ~isempty(labels_to_plot_4)
        [~, sort_idx] = sort([labels_to_plot_4.x]);
        labels_to_plot_4 = labels_to_plot_4(sort_idx);
        
        y_lims = ylim(ax4);
        y_min = y_lims(1); y_max = y_lims(2); y_range = y_max - y_min;
        x_lims = xlim(ax4); if isempty(x_lims), x_lims = [0 1]; end
        x_range = x_lims(2) - x_lims(1);
        
        % Constants
        MIN_X_DIST = x_range * 0.22; % Horizontal Collision Buffer
        DATA_BUFFER = y_range * 0.15; % Vertical distance from Data Point required
        
        placed_labels = struct('x', {}, 'is_top', {}, 'level', {}); 
        
        for k = 1:length(labels_to_plot_4)
            lbl = labels_to_plot_4(k);
            placed = false;
            level = 0; 
            
            % "Ping-Pong" Search: Try Top-0, then Bottom-0, then Top-1, etc.
            while ~placed
                % 1. CHECK TOP CANDIDATE
                y_top_cand = y_max - (0.08 + level*0.12) * y_range;
                
                % Check A: Does it cover the DATA POINT?
                if abs(y_top_cand - lbl.y_actual) > DATA_BUFFER
                    % Check B: Does it cover OTHER LABELS?
                    collision = false;
                    for p = 1:length(placed_labels)
                        pl = placed_labels(p);
                        % If same region (Top) and same level
                        if pl.is_top && pl.level == level
                            if abs(lbl.x - pl.x) < MIN_X_DIST
                                collision = true; break;
                            end
                        end
                    end
                    
                    if ~collision
                        % Valid! Place at Top
                        plot(ax4, [lbl.x, lbl.x], [lbl.y_actual, y_top_cand], ':', 'Color', [lbl.color, 0.6], 'LineWidth', 1.0);
                        text(ax4, lbl.x, y_top_cand, lbl.str, 'Color', lbl.color, 'FontSize', 8, 'FontWeight', 'bold', ...
                             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                             'BackgroundColor', [1 1 1 1.0], 'EdgeColor', 'none', 'Margin', 2);
                        placed_labels(end+1) = struct('x', lbl.x, 'is_top', true, 'level', level); %#ok<AGROW>
                        placed = true;
                        continue; % Done with this label
                    end
                end
                
                % 2. CHECK BOTTOM CANDIDATE (Fallback)
                y_bot_cand = y_min + (0.08 + level*0.12) * y_range;
                
                % Check A: Does it cover the DATA POINT?
                if abs(y_bot_cand - lbl.y_actual) > DATA_BUFFER
                    % Check B: Collision with LABELS
                    collision = false;
                    for p = 1:length(placed_labels)
                        pl = placed_labels(p);
                        if ~pl.is_top && pl.level == level
                            if abs(lbl.x - pl.x) < MIN_X_DIST
                                collision = true; break;
                            end
                        end
                    end
                    
                    if ~collision
                        % Valid! Place at Bottom
                        plot(ax4, [lbl.x, lbl.x], [lbl.y_actual, y_bot_cand], ':', 'Color', [lbl.color, 0.6], 'LineWidth', 1.0);
                        text(ax4, lbl.x, y_bot_cand, lbl.str, 'Color', lbl.color, 'FontSize', 8, 'FontWeight', 'bold', ...
                             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                             'BackgroundColor', [1 1 1 1.0], 'EdgeColor', 'none', 'Margin', 2);
                        placed_labels(end+1) = struct('x', lbl.x, 'is_top', false, 'level', level); %#ok<AGROW>
                        placed = true;
                        continue;
                    end
                end
                
                % If both blocked, increase level (move inwards)
                level = level + 1;
            end
        end
    end
    
    xlabel(ax4, 'Noise Multiplier (x baseline)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel(ax4, 'RMSE', 'FontSize', 10, 'FontWeight', 'bold');
    title(ax4, 'RMSE Growth: y, dy, θ, dθ', 'FontSize', 11, 'FontWeight', 'bold');
    
    hL4 = legend(plot_handles_remaining, 'NumColumns', 2, 'FontSize', 7);
    leg_x4 = (ax4_pos(1) + ax4_pos(3)) - 0.14; leg_y4 = ax4_pos(2) - 0.23;
    set(hL4, 'Position', [leg_x4, leg_y4, 0.14, 0.14], 'Box', 'on');
    set(ax4, 'Position', ax4_pos);
    if ~isempty(x_all), xlim(ax4, [min(x_all) * 0.95, max(x_all) * 1.05]); end
else
    text(ax4, 0.5, 0.5, 'No divergence data', 'HorizontalAlignment', 'center'); axis(ax4, 'off');
end

end