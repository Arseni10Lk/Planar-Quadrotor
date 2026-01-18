function plot_quadrotor_separate_windows_exact(time, state_data, output_data, C, errors)
% PLOT_QUADROTOR_SEPARATE_WINDOWS_EXACT - Creates exact copies of each quadrotor plot in separate windows
% Each figure contains the EXACT same content as the original subplots, WITHOUT ideal lines

fprintf('\n== Creating EXACT Separate Windows for Quadrotor State Estimation ===\n');

% Force light theme
set(0, 'DefaultFigureColor', 'white');

screen_size = get(0,'ScreenSize');

%% 1. 2D TRAJECTORY - EXACT copy of ax_traj (without ideal) WITH ORIGINAL LEGEND
fprintf('Creating Figure 1: 2D Trajectory...\n');
fig1 = figure('Name','Planar Quadrotor: 2D Trajectory',...
              'NumberTitle','off',...
              'Position',[50, 50, 800, 700],...  % Increased size
              'Color','white');

ax_traj = gca;
hold(ax_traj, 'on');
grid(ax_traj, 'on');
box(ax_traj, 'on');
axis(ax_traj, 'equal');

% Plot data WITHOUT ideal (grey dotted line removed)
% In original: plot(..., 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5) for ideal - REMOVED

% EKF Estimate - EXACT from original with proper legend
h_ekf = plot(ax_traj, state_data.estimate(:,1), state_data.estimate(:,3), 'b-', ...
     'LineWidth', 2.5);

% Real Trajectory - EXACT from original with proper legend
h_real = plot(ax_traj, state_data.real(:,1), state_data.real(:,3), 'r-', ...
     'LineWidth', 1.5);

% Add orientation bars (EXACT from original)
N = length(time);
num_bars = 15;
bar_indices = round(linspace(1, N, num_bars));
bar_length = 10.0;
arrow_length = 4.0;

for bar_idx = 1:num_bars
    i = bar_indices(bar_idx);
    x_pos = state_data.real(i, 1);
    y_pos = state_data.real(i, 3);
    theta = state_data.real(i, 5);
    
    % Bar coords
    bx = (bar_length/2) * cos(theta);
    by = (bar_length/2) * sin(theta);
    
    % Draw bar
    plot(ax_traj, [x_pos-bx, x_pos+bx], [y_pos-by, y_pos+by], ...
         'Color', [0.1 0.6 0.1], 'LineWidth', 5);
    
    % Draw Arrow
    quiver(ax_traj, x_pos, y_pos, ...
           arrow_length * cos(theta + pi/2), arrow_length * sin(theta + pi/2), ...
           'MaxHeadSize', 1.2, 'Color', [0.9 0.1 0.1], 'LineWidth', 3, ...
           'AutoScale', 'off');
    
    scatter(ax_traj, x_pos, y_pos, 20, [0.3 0.3 0.3], 'filled', 'o');
end

% Start/End Markers (EXACT from original)
h_start = scatter(ax_traj, state_data.real(1,1), state_data.real(1,3), ...
        50, [0 0.7 0.7], 'filled', '^', 'LineWidth', 1, 'MarkerEdgeColor', 'k');
h_end = scatter(ax_traj, state_data.real(end,1), state_data.real(end,3), ...
        50, [1 0.5 0], 'filled', 'v', 'LineWidth', 1, 'MarkerEdgeColor', 'k');

% Create custom legend like in the original
% The original has: Ideal (removed), EKF Estimate, Real Trajectory, Start, End
legend_handles = [h_ekf, h_real, h_start, h_end];
legend_labels = {'EKF Estimate', 'Real Trajectory', 'Start', 'End'};
legend(ax_traj, legend_handles, legend_labels, 'Location', 'best', 'FontSize', 9);

% Labels
xlabel(ax_traj, 'Horizontal Position, x (m)', 'FontWeight', 'bold', 'FontSize', 10);
ylabel(ax_traj, 'Altitude, y (m)', 'FontWeight', 'bold', 'FontSize', 10);
title(ax_traj, '2D Trajectory', 'FontWeight', 'bold', 'FontSize', 11);

% Fix Axis Limits to avoid crunching (EXACT from original)
x_margin = bar_length * 0.8;
y_margin = bar_length * 0.8;
xlim(ax_traj, [min(state_data.estimate(:,1))-x_margin, max(state_data.estimate(:,1))+x_margin]);
ylim(ax_traj, [min(state_data.estimate(:,3))-y_margin, max(state_data.estimate(:,3))+y_margin]);

%% 2. RMSE INFORMATION TABLE - EXACT copy of ax_info
fprintf('Creating Figure 2: RMSE Information Table...\n');
fig2 = figure('Name','RMSE Information Table',...
              'NumberTitle','off',...
              'Position',[100, 100, 500, 350],...
              'Color','white');

axis off;

if exist('errors', 'var') && isfield(errors, 'rmse_states')
    rmse_text = {'RMSE Values:', '----------------'};
    
    state_labels = {'x (Pos):   ', 'dx(Vel):  ', ' y (Pos):  ',...
                    'dy(Vel):  ', ' θ (Ang):  ', 'dθ(Rate): '};
    
    for i = 1:6
        if i <= 4
            val = errors.rmse_states(i);
            unit = 'm';
            if mod(i,2) == 0
                unit = 'm/s';
            end
        else
            val = rad2deg(errors.rmse_states(i));
            unit = 'deg';
            if i == 6
                unit = 'deg/s';
            end
        end
        rmse_text{end+1} = sprintf('%s%7.4f %s', state_labels{i}, val, unit);
    end
else
    rmse_text = {'No RMSE data'};
end

% Draw text in the MIDDLE (y=0.5) to avoid overlapping with top/bottom legends
text(0.5, 0.5, rmse_text, ...
     'FontName', 'FixedWidth', 'FontSize', 9, ...
     'HorizontalAlignment', 'center', ...
     'VerticalAlignment', 'middle', ...
     'BackgroundColor', [0.98 0.98 0.98], ...
     'EdgeColor', [0.9 0.9 0.9], 'Margin', 6);

title('RMSE Information', 'FontWeight', 'bold', 'FontSize', 11);

%% 3. INDIVIDUAL STATE PLOTS - 6 Separate Windows (EXACT copies, WITHOUT ideal)
fprintf('Creating Figures 3-8: Individual State Plots...\n');

% FIXED: Correct state_config cell array
state_config = {
    {'x (m)', 'Horizontal Position'};
    {'dx/dt (m/s)', 'Horizontal Velocity'};
    {'y (m)', 'Vertical Position'};
    {'dy/dt (m/s)', 'Vertical Velocity'};
    {'θ (deg)', 'Pitch Angle'};
    {'dθ/dt (deg/s)', 'Pitch Rate'}};

blue_lw = 2.0;
red_lw = 3.0;

for idx = 1:6
    fig_state = figure('Name', sprintf('State Plot: %s', state_config{idx}{2}),...
                       'NumberTitle','off',...
                       'Position',[50+idx*60, 50+idx*60, 750, 550],...  % Increased size
                       'Color','white');
    
    ax_state = gca;
    hold(ax_state,'on');
    grid(ax_state,'on');
    box(ax_state,'on');
    
    if idx >= 5
        conv = @rad2deg;
    else
        conv = @(x)x;
    end
    
    % Plot WITHOUT ideal (grey dotted line) - removed the first plot
    % Original had: h1 = plot(..., 'Ideal') - REMOVED
    
    % Only plot Real and EKF Estimate (EXACT from original)
    h_real = plot(ax_state, time, conv(state_data.real(:,idx)), '-', ...
             'Color', [0.85 0.2 0.2], 'LineWidth', red_lw, 'DisplayName', 'Real');
    
    h_ekf = plot(ax_state, time, conv(state_data.estimate(:,idx)), '-', ...
             'Color', [0.2 0.3 0.9], 'LineWidth', blue_lw, 'DisplayName', 'EKF');
    
    ylabel(ax_state, state_config{idx}{1}, 'FontSize', 9);
    title(ax_state, state_config{idx}{2}, 'FontWeight', 'bold', 'FontSize', 10);
    xlabel('Time (s)', 'FontSize', 9);
    
    set(ax_state,'GridAlpha',0.3);
    legend([h_real, h_ekf], {'Real', 'EKF'}, 'Location', 'best', 'FontSize', 9);
end

%% 4. STATE PLOTS LEGEND - Separate Window (optional)
fprintf('Creating Figure 9: State Plots Legend...\n');
fig_legend = figure('Name','State Plots Legend',...
                    'NumberTitle','off',...
                    'Position',[600, 600, 450, 250],...  % Increased size
                    'Color','white');

axis off;

legend_text = {'State Plots Legend:',...
               '-------------------',...
               '• Real: Red solid line',...
               '• EKF Estimate: Blue solid line',...
               '',...
               'Note: Ideal (grey dotted) line',...
               'has been removed as requested.'};

text(0.1, 0.8, legend_text, 'FontSize', 11, 'VerticalAlignment', 'top',...
     'FontWeight', 'bold');

%% 5. ERROR EVOLUTION PLOT - EXACT copy of ax_error
fprintf('Creating Figure 10: Estimation Errors...\n');
fig_error = figure('Name','Estimation Errors Evolution',...
                   'NumberTitle','off',...
                   'Position',[700, 50, 850, 550],...  % Increased size
                   'Color','white');

ax_error = gca;
hold(ax_error, 'on');
grid(ax_error, 'on');
box(ax_error, 'on');

if exist('errors', 'var') && isfield(errors, 'states_real_VS_estimate')
    % Data Prep (EXACT from original)
    err_k = abs(errors.states_real_VS_estimate);
    err_k(:,5:6) = rad2deg(err_k(:,5:6));
    err_r = abs(errors.states_real_VS_running);
    err_r(:,5:6) = rad2deg(err_r(:,5:6));
    
    colors = lines(6);
    names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    max_y = 0;
    
    % Store handles for legend
    plot_handles = [];
    legend_labels = {};
    
    for i = 1:6
        w = min(40, floor(length(time)/8));
        
        % Plot Running (Dashed) - Plot FIRST so it appears in legend but stays behind if needed
        h_run = plot(ax_error, time, movmean(err_r(:,i), w), '--', ...
             'Color', colors(i,:), 'LineWidth', 1.0);
        
        % Plot EKF (Solid)
        ekf_line = movmean(err_k(:,i), w);
        max_y = max(max_y, max(ekf_line));
        h_ekf = plot(ax_error, time, ekf_line, '-', ...
             'Color', colors(i,:), 'LineWidth', 1.5);
        
        % Store handles for legend (only once per state)
        if i == 1
            plot_handles = [h_run, h_ekf];
            legend_labels = {[names{i} ' (Running Mean)'], [names{i} ' (EKF)']};
        end
    end
    
    if max_y > 0
        ylim(ax_error, [0, max_y * 1.3]);
    end
    
    xlabel(ax_error, 'Time (s)', 'FontSize', 10);
    ylabel(ax_error, 'Absolute Error', 'FontSize', 10);
    title(ax_error, 'Estimation Errors Evolution', 'FontWeight', 'bold', 'FontSize', 11);
    
    % Create compact legend showing just one example
    legend(plot_handles, legend_labels, 'Location', 'best', 'FontSize', 9);
else
    text(0.5, 0.5, 'No Error Data Available',...
         'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    axis(ax_error, 'off');
end

fprintf('Created 10 separate figures for quadrotor state estimation.\n');
drawnow;
end