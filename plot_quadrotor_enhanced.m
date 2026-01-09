function plot_quadrotor_enhanced(time, state_data, output_data, C, errors)


% Force light theme
set(0, 'DefaultFigureColor', 'white');

% Get screen size
screen_size = get(0, 'ScreenSize');
fig_width = min(1400, screen_size(3) * 0.9);
fig_height = min(1000, screen_size(4) * 0.9);
fig_x = (screen_size(3) - fig_width) / 2;
fig_y = (screen_size(4) - fig_height) / 2;

% Create figure
fig = figure('Name', 'Planar Quadrotor: Final State Estimation', ...
             'NumberTitle', 'off', ...
             'Position', [fig_x, fig_y, fig_width, fig_height], ...
             'Color', 'white');

%% ================= 1. CLEAN 2D TRAJECTORY (NO ORIENTATION KEY) =================
ax_traj = subplot(4, 4, [1 2 5 6]);
hold(ax_traj, 'on'); 
grid(ax_traj, 'on'); 
box(ax_traj, 'on');
axis(ax_traj, 'equal');

N = length(time);

% === PLOT TRAJECTORIES ===
% 1. Ideal trajectory (black dashed)
plot(ax_traj, state_data.clean(:,1), state_data.clean(:,3), '--', ...
     'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'DisplayName', 'Ideal Trajectory');

% 2. EKF estimated trajectory (blue solid)
plot(ax_traj, state_data.estimate(:,1), state_data.estimate(:,3), 'b-', ...
     'LineWidth', 2.5, 'DisplayName', 'EKF Estimate');

% 3. Real trajectory (red solid)
plot(ax_traj, state_data.real(:,1), state_data.real(:,3), 'r-', ...
     'LineWidth', 1.5, 'DisplayName', 'Real Trajectory (Noisy)');

% === ADD CLEAN ORIENTATION BARS (NO CLUTTER) ===
num_bars = 10;
bar_indices = round(linspace(1, N, num_bars));

% Scale parameters
bar_length = 10.0;    % Visible but not overwhelming
arrow_length = 4.0;

for bar_idx = 1:num_bars
    i = bar_indices(bar_idx);
    x_pos = state_data.estimate(i, 1);
    y_pos = state_data.estimate(i, 3);
    theta = state_data.estimate(i, 5);
    
    % Orientation calculations
    thrust_angle = theta;               % Thrust direction
    body_angle = theta + pi/2;          % Body direction
    
    % === CREATE CLEAN ORIENTATION BAR ===
    bar_start_x = x_pos - (bar_length/2) * cos(body_angle);
    bar_start_y = y_pos - (bar_length/2) * sin(body_angle);
    bar_end_x = x_pos + (bar_length/2) * cos(body_angle);
    bar_end_y = y_pos + (bar_length/2) * sin(body_angle);
    
    % Draw the body bar
    if bar_idx == 1
        plot(ax_traj, [bar_start_x, bar_end_x], [bar_start_y, bar_end_y], ...
             'Color', [0.1 0.6 0.1], 'LineWidth', 5, ...
             'DisplayName', 'Drone Body');
    else
        plot(ax_traj, [bar_start_x, bar_end_x], [bar_start_y, bar_end_y], ...
             'Color', [0.1 0.6 0.1], 'LineWidth', 5, ...
             'HandleVisibility', 'off');
    end
    
    % === ADD THRUST ARROW ===
    if bar_idx == 1
        quiver(ax_traj, x_pos, y_pos, ...
               arrow_length * cos(thrust_angle), arrow_length * sin(thrust_angle), ...
               'MaxHeadSize', 1.2, 'Color', [0.9 0.1 0.1], 'LineWidth', 3, ...
               'AutoScale', 'off', 'DisplayName', 'Thrust');
    else
        quiver(ax_traj, x_pos, y_pos, ...
               arrow_length * cos(thrust_angle), arrow_length * sin(thrust_angle), ...
               'MaxHeadSize', 1.2, 'Color', [0.9 0.1 0.1], 'LineWidth', 3, ...
               'AutoScale', 'off', 'HandleVisibility', 'off');
    end
    
    % Minimal center marker
    scatter(ax_traj, x_pos, y_pos, 20, [0.3 0.3 0.3], 'filled', 'o', ...
            'HandleVisibility', 'off');
end

% === START AND END MARKERS ===
% Start: Cyan (not green)
scatter(ax_traj, state_data.clean(1,1), state_data.clean(1,3), ...
        180, [0 0.7 0.7], 'filled', '^', 'LineWidth', 2, ...  % Cyan
        'MarkerEdgeColor', 'k', 'DisplayName', 'Start');

% End: Orange
scatter(ax_traj, state_data.clean(end,1), state_data.clean(end,3), ...
        180, [1 0.5 0], 'filled', 'v', 'LineWidth', 2, ...
        'MarkerEdgeColor', 'k', 'DisplayName', 'End');

% Labels and legend
xlabel(ax_traj, 'Horizontal Position, x (m)', 'FontWeight', 'bold', 'FontSize', 11);
ylabel(ax_traj, 'Altitude, y (m)', 'FontWeight', 'bold', 'FontSize', 11);
title(ax_traj, '2D Trajectory with Drone Orientation', ...
      'FontWeight', 'bold', 'FontSize', 12);
legend(ax_traj, 'Location', 'best', 'FontSize', 9);

% Auto-adjust axis
x_margin = bar_length * 0.5;
y_margin = bar_length * 0.5;
x_lim = [min(state_data.estimate(:,1))-x_margin, max(state_data.estimate(:,1))+x_margin];
y_lim = [min(state_data.estimate(:,3))-y_margin, max(state_data.estimate(:,3))+y_margin];
axis(ax_traj, [x_lim, y_lim]);

%% ================= 2. RMSE SUMMARY =================
ax_info = subplot(4, 4, [3 4]);
axis(ax_info, 'off');

if exist('errors', 'var') && isfield(errors, 'rmse_states')
    rmse_text = {'ESTIMATION ACCURACY (RMSE)', ...
                 '===============================', ...
                 ''};
    
    state_labels = {
        'Horizontal Position (x)    : ';
        'Horizontal Velocity (dx)   : ';
        'Vertical Position (y)      : ';
        'Vertical Velocity (dy)     : ';
        'Pitch Angle (θ)           : ';
        'Pitch Rate (dθ)           : '
    };
    
    for i = 1:6
        if i <= 4
            value = errors.rmse_states(i);
            if i == 1 || i == 3
                unit = ' m';
            else
                unit = ' m/s';
            end
        else
            value = rad2deg(errors.rmse_states(i));
            if i == 5
                unit = '°';
            else
                unit = '°/s';
            end
        end
        rmse_text{end+1} = sprintf('%s%8.4f%s', state_labels{i}, value, unit);
    end
    
    % Performance summary
    rmse_text{end+1} = '';
    rmse_text{end+1} = 'Performance Summary:';
    rmse_text{end+1} = sprintf('  Position Error: %8.4f m', mean(errors.rmse_states([1,3])));
    rmse_text{end+1} = sprintf('  Velocity Error: %8.4f m/s', mean(errors.rmse_states([2,4])));
    rmse_text{end+1} = sprintf('  Angle Error:    %8.4f°', rad2deg(mean(errors.rmse_states([5,6]))));
    
    % Rating
    avg_error = mean(errors.rmse_states(1:4));
    if avg_error < 0.01
        rating = 'EXCELLENT';
        color = [0 0.6 0];
    elseif avg_error < 0.05
        rating = 'GOOD';
        color = [0 0.4 0.8];
    else
        rating = 'FAIR';
        color = [1 0.5 0];
    end
    
    rmse_text{end+1} = '';
    rmse_text{end+1} = sprintf('Overall: %s', rating);
    
else
    rmse_text = {'No RMSE data available'};
    color = [0.3 0.3 0.3];
end

text(0.05, 0.95, rmse_text, ...
     'FontName', 'FixedWidth', 'FontSize', 9.5, ...
     'VerticalAlignment', 'top', ...
     'BackgroundColor', [0.97 0.97 0.97], ...
     'EdgeColor', [0.8 0.8 0.8], 'LineWidth', 1);

if exist('errors', 'var') && isfield(errors, 'rmse_states')
    rating_line = findobj(ax_info, 'String', sprintf('Overall: %s', rating));
    if ~isempty(rating_line)
        set(rating_line, 'Color', color, 'FontWeight', 'bold', 'FontSize', 10);
    end
end

title(ax_info, 'Estimation Performance', ...
      'FontWeight', 'bold', 'FontSize', 11);

%% ================= 3. ALL 6 STATE PLOTS (RED LINES 70% THICKER) =================
state_positions = [9, 10, 11, 12, 13, 14];

state_config = {
    {'x (m)', 'Horizontal Position'};
    {'dx/dt (m/s)', 'Horizontal Velocity'};
    {'y (m)', 'Vertical Position'};
    {'dy/dt (m/s)', 'Vertical Velocity'};
    {'θ (deg)', 'Pitch Angle'};
    {'dθ/dt (deg/s)', 'Pitch Rate'};
};

% === KEY: RED LINES THICKER THAN BLUE ===

blue_line_width = 2.0;          
red_line_width = blue_line_width * 1.7; 

for state_idx = 1:6
    ax_state = subplot(4, 4, state_positions(state_idx));
    hold(ax_state, 'on'); 
    grid(ax_state, 'on'); 
    box(ax_state, 'on');
    
    % Convert angles
    if state_idx == 5 || state_idx == 6
        ideal_vals = rad2deg(state_data.clean(:, state_idx));
        real_vals = rad2deg(state_data.real(:, state_idx));
        ekf_vals = rad2deg(state_data.estimate(:, state_idx));
    else
        ideal_vals = state_data.clean(:, state_idx);
        real_vals = state_data.real(:, state_idx);
        ekf_vals = state_data.estimate(:, state_idx);
    end
    
    % === PLOT WITH 70% THICKER RED LINES ===
    % 1. Ideal values (subtle gray dots)
    plot(ax_state, time(1:20:end), ideal_vals(1:20:end), '.', ...
         'Color', [0.5 0.5 0.5], 'MarkerSize', 6, 'DisplayName', 'Ideal');
    
    % 2. Real values (RED - 70% THICKER THAN BLUE)
    plot(ax_state, time, real_vals, '-', ...
         'Color', [0.85 0.2 0.2], 'LineWidth', red_line_width, ...  % 3.4 (70% thicker)
         'DisplayName', 'Real State');
    
    % 3. EKF estimate (BLUE - baseline thickness)
    plot(ax_state, time, ekf_vals, '-', ...
         'Color', [0.2 0.3 0.9], 'LineWidth', blue_line_width, ...  % 2.0
         'DisplayName', 'EKF Estimate');
    
    % Labels
    xlabel(ax_state, 'Time (s)', 'FontSize', 9);
    ylabel(ax_state, state_config{state_idx, 1}{1}, 'FontSize', 9);
    title(ax_state, state_config{state_idx, 1}{2}, ...
          'FontWeight', 'bold', 'FontSize', 10);
    
    % Legend only on first
    if state_idx == 1
        legend(ax_state, {'Ideal', 'Real State', 'EKF Estimate'}, ...
               'Location', 'best', 'FontSize', 8);
    end
    
    set(ax_state, 'GridAlpha', 0.3);
end

%% ================= 4. ERROR EVOLUTION PLOT =================
ax_error = subplot(4, 4, [15 16]);
hold(ax_error, 'on'); grid(ax_error, 'on'); box(ax_error, 'on');

if exist('errors', 'var') && isfield(errors, 'states_real_VS_estimate')
    abs_errors = abs(errors.states_real_VS_estimate);
    abs_errors(:,5) = rad2deg(abs_errors(:,5));
    abs_errors(:,6) = rad2deg(abs_errors(:,6));
    
    colors = lines(6);
    state_names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    
    for i = 1:6
        window = min(40, floor(length(time)/8));
        smooth_err = movmean(abs_errors(:, i), window);
        plot(ax_error, time, smooth_err, ...
             'Color', colors(i, :), 'LineWidth', 1.8, ...
             'DisplayName', state_names{i});
    end
    
    xlabel(ax_error, 'Time (s)', 'FontSize', 10);
    ylabel(ax_error, 'Absolute Error', 'FontSize', 10);
    title(ax_error, 'Estimation Errors Over Time', 'FontWeight', 'bold', 'FontSize', 11);
    legend(ax_error, 'Location', 'best', 'FontSize', 8, 'NumColumns', 3);
    
    plot(ax_error, [time(1) time(end)], [0 0], 'k:', ...
         'LineWidth', 0.5, 'HandleVisibility', 'off');
else
    text(ax_error, 0.5, 0.5, 'Error data not available', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    axis(ax_error, 'off');
end

%% ================= FINAL TOUCHES =================
% Main title
sgtitle({'Planar Quadrotor: Final State Estimation Results', ...
         sprintf('Clean visualization', ...
                 red_line_width/blue_line_width)}, ...
        'FontSize', 14, 'FontWeight', 'bold');

% Clean annotation
annotation('textbox', [0.02, 0.02, 0.2, 0.04], ...
           'String', {'Final Visualization', ...
                     sprintf('Generated: %s', datestr(now))}, ...
           'FontSize', 8, 'FontWeight', 'bold', ...
           'EdgeColor', 'none', 'BackgroundColor', 'none', ...
           'FitBoxToText', 'on');

drawnow;

% Console summary
if exist('errors', 'var') && isfield(errors, 'rmse_states')
    fprintf('\n=== FINAL VISUALIZATION SUMMARY ===\n');
    fprintf('Trajectory: Clean, no orientation key clutter\n');
    fprintf('State plots: \n', ...
            red_line_width/blue_line_width);
    fprintf('Start marker: Cyan (not green)\n');
    fprintf('Line thickness: Red = %.1f, Blue = %.1f\n', ...
            red_line_width, blue_line_width);
    fprintf('==================================\n');
end
end