function plot_robustness_separate_windows(rmse_matrix, noise_matrix, divergence_data, case_name)
% PLOT_ROBUSTNESS_SEPARATE_WINDOWS - Creates exact copies of each robustness plot in separate windows
% Each figure contains the EXACT same content as the original subplots
% SAVES figures to /Report folder with case_name suffix.

fprintf('\n== Creating EXACT Separate Windows for Robustness Plots (%s) ===\n', case_name);

% Force light theme
set(0, 'DefaultFigureColor', 'white');

% Create Report Directory
report_dir = fullfile(pwd, 'Report');
if ~exist(report_dir, 'dir')
    mkdir(report_dir);
end

%% 1. RMSE BAR CHART (EKF Only) - EXACT copy of ax1 - FIXED VERSION
fprintf('Creating Figure 1: EKF RMSE Across Noise Conditions...\n');
fig1 = figure('Name', sprintf('EKF RMSE Across Noise Conditions (%s)', case_name),...
              'NumberTitle','off',...
              'Position',[50, 50, 600, 450],...
              'Color','white');

% EXACT code from plot_robustness_results.m for ax1 - FIXED!
% The original has 6 states (x, dx, y, dy, θ, dθ) and 4 noise cases
% So rmse_matrix should be 4x6 (cases x states), but bar() will show 6 groups with 4 bars each
bar(rmse_matrix','grouped');  % TRANSPOSE to get correct orientation
xlabel('State','FontSize',10,'FontWeight','bold');
ylabel('RMSE (EKF)','FontSize',10,'FontWeight','bold');
title(sprintf('EKF RMSE Across Noise Conditions - %s', case_name),'FontSize',11,'FontWeight','bold');
set(gca,'XTickLabel',{'x','dx','y','dy','θ','dθ'});
grid on;
legend({'Optimal','Regular','High','Very High'},...
       'Location','northoutside','Orientation','horizontal','FontSize',8);

% SAVE FIGURE
saveas(fig1, fullfile(report_dir, sprintf('robustness_rmse-%s.png', case_name)));

%% 2. NOISE LEVELS - EXACT copy of ax2
fprintf('Creating Figure 2: Noise Levels...\n');
fig2 = figure('Name', sprintf('Noise Levels (%s)', case_name),...
              'NumberTitle','off',...
              'Position',[100, 100, 600, 450],...
              'Color','white');

% EXACT code from plot_robustness_results.m for ax2
bar(noise_matrix);
xlabel('Case','FontSize',10,'FontWeight','bold');
ylabel('Noise Amplitude','FontSize',10,'FontWeight','bold');
title(sprintf('Noise Levels - %s', case_name),'FontSize',11,'FontWeight','bold');
set(gca,'XTickLabel',{'Optimal','Regular','High','Very High'});
grid on;
legend({'Process','Measurement'},...
       'Location','northoutside','Orientation','horizontal','FontSize',8);

% SAVE FIGURE
saveas(fig2, fullfile(report_dir, sprintf('noise_levels-%s.png', case_name)));

%% 3. RMSE GROWTH FOR X AND DX (LOG SCALE) - EXACT copy of ax3
if ~isempty(divergence_data.multipliers) && ~isempty(divergence_data.rmse_values)
    fprintf('Creating Figure 3: RMSE Growth for x and dx (Log Scale)...\n');
    fig3 = figure('Name', sprintf('RMSE Growth: x and dx (Log Scale) (%s)', case_name),...
                  'NumberTitle','off',...
                  'Position',[150, 150, 700, 550],...
                  'Color','white');
    
    ax3 = gca;
    ax3_pos = get(ax3,'Position');
    hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
    set(ax3,'YScale','log');
    
    % EXACT code from plot_robustness_results.m for ax3
    labels_to_plot_3 = struct('x',{},'y_actual',{},'str',{},'color',{});
    
    colors = lines(6);
    state_names = {'x','dx','y','dy','θ','dθ'};
    xdx_indices = [1,2];
    plot_handles_xdx = [];
    
    for plot_idx = 1:2
        state_idx = xdx_indices(plot_idx);
        data_state_name = divergence_data.state_names{state_idx};
        
        % --- EKF (SOLID) ---
        div_field = ['div_point_' data_state_name];
        actual_div_field = ['actually_diverged_' data_state_name];
        
        if isfield(divergence_data, div_field)
            div_point = divergence_data.(div_field);
            actually_diverged = divergence_data.(actual_div_field);
            x_all = divergence_data.multipliers;
            y_all = divergence_data.rmse_values(:, state_idx);
            y_all_pos = y_all; 
            y_all_pos(y_all_pos <= 0) = 1e-10;
            
            if actually_diverged
                idx = find(x_all <= div_point);
                if ~isempty(idx)
                    h = plot(ax3, x_all(idx), y_all_pos(idx), '-', ...
                           'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                           'DisplayName', sprintf('%s (EKF)', state_names{state_idx}));
                    final_y = y_all_pos(idx(end));
                    
                    % BLUE/MAROON DOT at end point
                    plot(ax3, div_point, final_y, 'o', ...
                         'MarkerSize', 8, 'MarkerFaceColor', colors(state_idx,:),...
                         'MarkerEdgeColor', 'k');
                    
                    xline(ax3, div_point, '--', 'Color', colors(state_idx,:),...
                          'LineWidth', 1.0, 'Alpha', 0.5);
                    
                    labels_to_plot_3(end+1) = struct('x', div_point,...
                                                   'y_actual', final_y,...
                                                   'str', sprintf('%.1fx', div_point),...
                                                   'color', colors(state_idx,:));
                end
            else
                h = plot(ax3, x_all, y_all_pos, '-', ...
                       'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                       'DisplayName', sprintf('%s (EKF)', state_names{state_idx}));
                
                % SQUARE marker at end point for stable states
                plot(ax3, x_all(end), y_all_pos(end), 's', ...
                     'MarkerSize', 8, 'MarkerFaceColor', colors(state_idx,:),...
                     'MarkerEdgeColor', 'k');
            end
            plot_handles_xdx = [plot_handles_xdx, h];
        end
        
        % RUNNING FILTER (DASHED)
        if isfield(divergence_data,'rmse_values_running')
            div_field_run = ['div_point_running_' data_state_name];
            actual_div_field_run = ['actually_diverged_running_' data_state_name];
            y_all_run = divergence_data.rmse_values_running(:, state_idx);
            y_all_run_pos = y_all_run; 
            y_all_run_pos(y_all_run_pos <= 0) = 1e-10;
            
            div_point_run = divergence_data.(div_field_run);
            actually_diverged_run = divergence_data.(actual_div_field_run);
            
            if actually_diverged_run
                idx = find(x_all <= div_point_run);
                if ~isempty(idx)
                    h_run = plot(ax3, x_all(idx), y_all_run_pos(idx), '--', ...
                               'Color', colors(state_idx,:), 'LineWidth', 1.5, ...
                               'DisplayName', sprintf('%s (Running Mean)', state_names{state_idx}));
                    final_y_run = y_all_run_pos(idx(end));
                    
                    % X marker for running filter
                    plot(ax3, div_point_run, final_y_run, 'x', ...
                         'MarkerSize', 8, 'Color', colors(state_idx,:), 'LineWidth', 1.5);
                    
                    xline(ax3, div_point_run, '--', 'Color', colors(state_idx,:),...
                          'LineWidth', 0.8, 'Alpha', 0.4);
                    
                    labels_to_plot_3(end+1) = struct('x', div_point_run,...
                                                   'y_actual', final_y_run,...
                                                   'str', sprintf('%.1f', div_point_run),...
                                                   'color', colors(state_idx,:)*0.8);
                end
            else
                h_run = plot(ax3, x_all, y_all_run_pos, '--', ...
                           'Color', colors(state_idx,:), 'LineWidth', 1.5, ...
                           'DisplayName', sprintf('%s (Running Mean)', state_names{state_idx}));
            end
            if length(plot_handles_xdx) < 4
                plot_handles_xdx = [plot_handles_xdx, h_run];
            end
        end
    end
    
    % Label stacking logic (EXACT from original)
    if ~isempty(labels_to_plot_3)
        [~, sort_idx] = sort([labels_to_plot_3.x]);
        labels_to_plot_3 = labels_to_plot_3(sort_idx);
        
        y_lims = ylim(ax3);
        y_min_log = log10(y_lims(1)); 
        y_max_log = log10(y_lims(2)); 
        y_range_log = y_max_log - y_min_log;
        
        x_lims = xlim(ax3); 
        if isempty(x_lims), x_lims = [0 1]; end
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
                    placed_labels(end+1) = struct('x', lbl.x, 'row', row);
                    pct_up = 0.05 + (row * 0.12);
                    y_pos = 10^(y_min_log + pct_up * y_range_log);
                    
                    plot(ax3, [lbl.x, lbl.x], [lbl.y_actual, y_pos], ':',...
                         'Color', [lbl.color, 0.6], 'LineWidth', 1.0);
                    text(ax3, lbl.x, y_pos, lbl.str, 'Color', lbl.color,...
                         'FontSize', 8, 'FontWeight', 'bold',...
                         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle',...
                         'BackgroundColor', [1 1 1], 'EdgeColor', 'none', 'Margin', 2);
                else
                    row = row + 1;
                end
            end
        end
    end
    
    xlabel(ax3, 'Noise Multiplier (x baseline)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel(ax3, 'RMSE (log scale)', 'FontSize', 10, 'FontWeight', 'bold');
    title(ax3, sprintf('RMSE Growth: x and dx (Log Scale) - %s', case_name), 'FontSize', 11, 'FontWeight', 'bold');
    
    if ~isempty(plot_handles_xdx)
        hL3 = legend(plot_handles_xdx, 'NumColumns', 1, 'FontSize', 8);
        set(hL3, 'Box', 'on');
    end
    
    if ~isempty(x_all)
        xlim(ax3, [min(x_all)*0.95, max(x_all)*1.05]);
    end
    grid(ax3, 'on'); grid(ax3, 'minor'); 
    ax3.MinorGridAlpha = 0.2; ax3.GridAlpha = 0.3;

    % SAVE FIGURE
    saveas(fig3, fullfile(report_dir, sprintf('rmse_growth_log-%s.png', case_name)));
end

%% 4. RMSE GROWTH FOR Y, DY, θ, dθ (LINEAR SCALE) - EXACT copy of ax4
if ~isempty(divergence_data.multipliers) && ~isempty(divergence_data.rmse_values)
    fprintf('Creating Figure 4: RMSE Growth for y, dy, θ, dθ...\n');
    fig4 = figure('Name', sprintf('RMSE Growth: y, dy, θ, dθ (%s)', case_name),...
                  'NumberTitle','off',...
                  'Position',[200, 200, 800, 550],...
                  'Color','white');
    
    ax4 = gca;
    ax4_pos = get(ax4,'Position');
    hold(ax4,'on'); grid(ax4,'on'); box(ax4,'on');
    
    % EXACT code from plot_robustness_results.m for ax4
    labels_to_plot_4 = struct('x',{},'y_actual',{},'str',{},'color',{});
    
    colors = lines(6);
    state_names = {'x','dx','y','dy','θ','dθ'};
    remaining_indices = [3,4,5,6];
    plot_handles_remaining = [];
    
    for plot_idx = 1:4
        state_idx = remaining_indices(plot_idx);
        data_state_name = divergence_data.state_names{state_idx};
        x_all = divergence_data.multipliers;
        y_all = divergence_data.rmse_values(:, state_idx);
        
        % EKF
        div_field = ['div_point_' data_state_name];
        actual_div_field = ['actually_diverged_' data_state_name];
        
        if isfield(divergence_data, div_field)
            div_point = divergence_data.(div_field);
            actually_diverged = divergence_data.(actual_div_field);
            
            if actually_diverged
                idx = find(x_all <= div_point);
                if ~isempty(idx)
                    h = plot(ax4, x_all(idx), y_all(idx), '-', ...
                           'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                           'DisplayName', sprintf('%s (EKF)', state_names{state_idx}));
                    final_y = y_all(idx(end));
                    
                    % BLUE/MAROON DOT at divergence point
                    plot(ax4, div_point, final_y, 'o',...
                         'MarkerSize', 6, 'MarkerFaceColor', colors(state_idx,:),...
                         'MarkerEdgeColor', 'k');
                    
                    xline(ax4, div_point, '--', 'Color', colors(state_idx,:),...
                          'LineWidth', 1.0, 'Alpha', 0.4);
                    
                    % STORE LABEL
                    labels_to_plot_4(end+1) = struct('x', div_point,...
                                                   'y_actual', final_y,...
                                                   'str', sprintf('%.1fx', div_point),...
                                                   'color', colors(state_idx,:));
                end
            else
                h = plot(ax4, x_all, y_all, '-', ...
                       'Color', colors(state_idx,:), 'LineWidth', 2.0, ...
                       'DisplayName', sprintf('%s (EKF)', state_names{state_idx}));
                xline(ax4, x_all(end), '--', 'Color', colors(state_idx,:),...
                      'LineWidth', 1.0, 'Alpha', 0.4);
            end
            plot_handles_remaining = [plot_handles_remaining, h];
        end
        
        % RUNNING
        if isfield(divergence_data,'rmse_values_running') && state_idx ~= 3 && state_idx ~= 4
            div_field_run = ['div_point_running_' data_state_name];
            actual_div_field_run = ['actually_diverged_running_' data_state_name];
            y_all_run = divergence_data.rmse_values_running(:, state_idx);
            div_point_run = divergence_data.(div_field_run);
            actually_diverged_run = divergence_data.(actual_div_field_run);
            
            if actually_diverged_run
                idx = find(x_all <= div_point_run);
                if ~isempty(idx)
                    h_run = plot(ax4, x_all(idx), y_all_run(idx), '--',...
                               'Color', colors(state_idx,:), 'LineWidth', 1.5,...
                               'DisplayName', sprintf('%s (Running Mean)', state_names{state_idx}));
                    final_y_run = y_all_run(idx(end));
                    
                    % X marker for running filter
                    plot(ax4, div_point_run, final_y_run, 'x',...
                         'MarkerSize', 6, 'Color', colors(state_idx,:), 'LineWidth', 1.5);
                    
                    xline(ax4, div_point_run, '--', 'Color', colors(state_idx,:),...
                          'LineWidth', 0.8, 'Alpha', 0.3);
                    
                    % STORE LABEL
                    labels_to_plot_4(end+1) = struct('x', div_point_run,...
                                                   'y_actual', final_y_run,...
                                                   'str', sprintf('%.1f', div_point_run),...
                                                   'color', colors(state_idx,:)*0.8);
                end
            else
                h_run = plot(ax4, x_all, y_all_run, '--',...
                           'Color', colors(state_idx,:), 'LineWidth', 1.5,...
                           'DisplayName', sprintf('%s (Running Mean)', state_names{state_idx}));
            end
            if length(plot_handles_remaining) < 8
                plot_handles_remaining = [plot_handles_remaining, h_run];
            end
        end
    end
    
    % Label placement logic (EXACT from original)
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
                        plot(ax4, [lbl.x, lbl.x], [lbl.y_actual, y_top_cand], ':',...
                             'Color', [lbl.color, 0.6], 'LineWidth', 1.0);
                        text(ax4, lbl.x, y_top_cand, lbl.str, 'Color', lbl.color,...
                             'FontSize', 8, 'FontWeight', 'bold',...
                             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle',...
                             'BackgroundColor', [1 1 1], 'EdgeColor', 'none', 'Margin', 2);
                        placed_labels(end+1) = struct('x', lbl.x, 'is_top', true, 'level', level);
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
                        plot(ax4, [lbl.x, lbl.x], [lbl.y_actual, y_bot_cand], ':',...
                             'Color', [lbl.color, 0.6], 'LineWidth', 1.0);
                        text(ax4, lbl.x, y_bot_cand, lbl.str, 'Color', lbl.color,...
                             'FontSize', 8, 'FontWeight', 'bold',...
                             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle',...
                             'BackgroundColor', [1 1 1], 'EdgeColor', 'none', 'Margin', 2);
                        placed_labels(end+1) = struct('x', lbl.x, 'is_top', false, 'level', level);
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
    title(ax4, sprintf('RMSE Growth: y, dy, θ, dθ - %s', case_name), 'FontSize', 11, 'FontWeight', 'bold');
    
    if ~isempty(plot_handles_remaining)
        hL4 = legend(plot_handles_remaining, 'NumColumns', 2, 'FontSize', 7);
        set(hL4, 'Box', 'on');
    end
    
    if ~isempty(x_all)
        xlim(ax4, [min(x_all)*0.95, max(x_all)*1.05]);
    end

    % SAVE FIGURE
    saveas(fig4, fullfile(report_dir, sprintf('rmse_growth_linear-%s.png', case_name)));
end

fprintf('Created 6 separate figures for robustness analysis and saved to Report folder.\n');
drawnow;
end