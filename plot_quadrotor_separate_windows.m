function plot_quadrotor_separate_windows(time, state_data, output_data, C, errors, case_name)
% PLOT_QUADROTOR_SEPARATE_WINDOWS - Creates exact copies of each quadrotor plot in separate windows
% Each figure contains the EXACT same content as the original subplots, WITHOUT ideal lines
% SAVES figures to /Report folder with case_name suffix.

fprintf('\n== Creating EXACT Separate Windows for Quadrotor State Estimation (%s) ===\n', case_name);

% Force light theme
set(0, 'DefaultFigureColor', 'white');

% Create Report Directory
report_dir = fullfile(pwd, 'Report');
if ~exist(report_dir, 'dir')
    mkdir(report_dir);
end

screen_size = get(0,'ScreenSize');

%% 1. 2D TRAJECTORY - EXACT copy of ax_traj (without ideal) WITH ORIGINAL LEGEND
fprintf('Creating Figure 1: 2D Trajectory...\n');
fig1 = figure('Name', sprintf('Planar Quadrotor: 2D Trajectory (%s)', case_name),...
              'NumberTitle','off',...
              'Position',[50, 50, 800, 700],...  % Increased size
              'Color','white');

ax_traj = gca;
hold(ax_traj, 'on');
grid(ax_traj, 'on');
box(ax_traj, 'on');
axis(ax_traj, 'equal');

% Plot data WITHOUT ideal (grey dotted line removed)

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
legend_handles = [h_ekf, h_real, h_start, h_end];
legend_labels = {'EKF Estimate', 'Real Trajectory', 'Start', 'End'};
legend(ax_traj, legend_handles, legend_labels, 'Location', 'best', 'FontSize', 9);

% Labels
xlabel(ax_traj, 'Horizontal Position, x (m)', 'FontWeight', 'bold', 'FontSize', 10);
ylabel(ax_traj, 'Altitude, y (m)', 'FontWeight', 'bold', 'FontSize', 10);
title(ax_traj, sprintf('2D Trajectory - %s', case_name), 'FontWeight', 'bold', 'FontSize', 11);

% Fix Axis Limits to avoid crunching (EXACT from original)
x_margin = bar_length * 0.8;
y_margin = bar_length * 0.8;
xlim(ax_traj, [min(state_data.estimate(:,1))-x_margin, max(state_data.estimate(:,1))+x_margin]);
ylim(ax_traj, [min(state_data.estimate(:,3))-y_margin, max(state_data.estimate(:,3))+y_margin]);

% SAVE FIGURE
saveas(fig1, fullfile(report_dir, sprintf('trajectory-%s.png', case_name)));

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

% Sanitize filenames for saving (replace / with _)
state_filenames = {'x', 'dx_dt', 'y', 'dy_dt', 'theta', 'dtheta_dt'};

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
    
    % Unit conversion function
    if idx >= 5
        conv = @rad2deg;
    else
        conv = @(x)x;
    end
    
    % --- Determine if this state has noisy sensor data ---
    % Mapping based on simulation_quadrotor.m:
    % State 3 (y)      <-- Output 1
    % State 5 (theta)  <-- Output 2
    % State 6 (dtheta) <-- Output 3
    
    has_sensor = false;
    sensor_col = 0;
    
    if idx == 3
        has_sensor = true;
        sensor_col = 1;
    elseif idx == 5
        has_sensor = true;
        sensor_col = 2;
    elseif idx == 6
        has_sensor = true;
        sensor_col = 3;
    end
    
    % --- Plotting ---
    
    % 1. Plot Sensor Data (if available) - plotted first (behind lines) or as scatter
    h_sens = [];
    if has_sensor
        % Extract noisy sensor data column and convert units if necessary
        sensor_vals = output_data.real(:, sensor_col);
        h_sens = plot(ax_state, time, conv(sensor_vals), '.', ...
            'Color', [0.2 0.7 0.2], 'MarkerSize', 6, 'DisplayName', 'Sensor');
    end

    % 2. Plot Real State
    h_real = plot(ax_state, time, conv(state_data.real(:,idx)), '-', ...
             'Color', [0.85 0.2 0.2], 'LineWidth', red_lw, 'DisplayName', 'Real');
    
    % 3. Plot EKF Estimate
    h_ekf = plot(ax_state, time, conv(state_data.estimate(:,idx)), '-', ...
             'Color', [0.2 0.3 0.9], 'LineWidth', blue_lw, 'DisplayName', 'EKF');
    
    ylabel(ax_state, state_config{idx}{1}, 'FontSize', 9);
    title(ax_state, state_config{idx}{2}, 'FontWeight', 'bold', 'FontSize', 10);
    xlabel('Time (s)', 'FontSize', 9);
    
    set(ax_state,'GridAlpha',0.3);
    
    % Update Legend based on whether sensor data exists
    if has_sensor
        legend([h_real, h_ekf, h_sens], {'Real', 'EKF', 'Sensor'}, 'Location', 'best', 'FontSize', 9);
    else
        legend([h_real, h_ekf], {'Real', 'EKF'}, 'Location', 'best', 'FontSize', 9);
    end

    % SAVE FIGURE
    saveas(fig_state, fullfile(report_dir, sprintf('state_%s-%s.png', state_filenames{idx}, case_name)));
end

%% 5. ERROR EVOLUTION PLOT - EXACT copy of ax_error
fprintf('Creating Figure 10: Estimation Errors...\n');
fig_error = figure('Name', sprintf('Estimation Errors Evolution (%s)', case_name),...
                   'NumberTitle','off',...
                   'Position',[700, 50, 850, 550],...  % Increased size
                   'Color','white');

ax_error = gca;
hold(ax_error, 'on'); grid(ax_error, 'on'); box(ax_error, 'on');

if exist('errors', 'var') && isfield(errors, 'states_real_VS_estimate')
    % Data Prep
    err_k = abs(errors.states_real_VS_estimate);
    err_k(:,5:6) = rad2deg(err_k(:,5:6));
    err_r = abs(errors.states_real_VS_running);
    err_r(:,5:6) = rad2deg(err_r(:,5:6));
    
    colors = lines(6);
    % Variable names for the solid lines
    names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    max_y = 0;
    
    ekf_handles = [];
    
    for i = 1:6
        w = min(40, floor(length(time)/8));
        
        % 1. Plot Running (Dashed) 
        % 'HandleVisibility','off' -> These actual colored dashed lines are NOT added to legend
        plot(ax_error, time, movmean(err_r(:,i), w), '--', ...
             'Color', colors(i,:), 'LineWidth', 1.0, ...
             'HandleVisibility', 'off');
        
        % 2. Plot EKF (Solid)
        ekf_line = movmean(err_k(:,i), w);
        max_y = max(max_y, max(ekf_line));
        
        h_ekf = plot(ax_error, time, ekf_line, '-', ...
             'Color', colors(i,:), 'LineWidth', 1.5);
             
        ekf_handles = [ekf_handles, h_ekf];
    end
    
    % --- CREATE DUMMY PLOT FOR LEGEND ---
    % Plot NaN values (invisible) but with Black Dashed style
    h_dummy_run = plot(ax_error, NaN, NaN, 'k--', 'LineWidth', 1.0);
    
    if max_y > 0
        ylim(ax_error, [0, max_y * 1.3]);
    end
    
    xlabel(ax_error, 'Time (s)', 'FontSize', 10);
    ylabel(ax_error, 'Absolute Error', 'FontSize', 10);
    title(ax_error, sprintf('Estimation Errors Evolution - %s', case_name), 'FontWeight', 'bold', 'FontSize', 11);
    
    % Combine handles: 6 Solid Colors + 1 Black Dashed
    final_handles = [ekf_handles, h_dummy_run];
    final_labels  = [names, {'Running Mean'}];
    
    legend(final_handles, final_labels, 'Location', 'best', 'FontSize', 9);
else
    text(0.5, 0.5, 'No Error Data Available',...
         'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    axis(ax_error, 'off');
end

% SAVE FIGURE
saveas(fig_error, fullfile(report_dir, sprintf('estimation_errors-%s.png', case_name)));

fprintf('Created 10 separate figures for quadrotor state estimation and saved to Report folder.\n');
drawnow;
end