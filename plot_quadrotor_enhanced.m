function plot_quadrotor_enhanced(time, state_data, output_data, C, errors)

% Forced light theme for consistent display
set(0, 'DefaultFigureColor', 'white');

% Get screen size for optimal window placement
screen_size = get(0, 'ScreenSize');
fig_width = min(1200, screen_size(3) * 0.85);
fig_height = min(800, screen_size(4) * 0.85);
fig_x = (screen_size(3) - fig_width) / 2;
fig_y = (screen_size(4) - fig_height) / 2;

% Create single figure window
fig = figure('Name', 'Quadrotor Simulation - Complete Results', ...
    'NumberTitle', 'off', ...
    'Position', [fig_x, fig_y, fig_width, fig_height], ...
    'Color', 'white');

%% ========== TOP: 2D TRAJECTORY WITH PITCH INDICATORS ==========
ax_traj = subplot(4, 3, [1 2 4 5]);
hold(ax_traj, 'on'); grid(ax_traj, 'on'); box(ax_traj, 'on');
axis(ax_traj, 'equal');

N = length(time);

% 1. Ideal trajectory ( gray line)
plot(ax_traj, state_data.clean(:,1), state_data.clean(:,3), ':', ...
    'Color', [0.7 0.7 0.7], 'LineWidth', 1, 'DisplayName', 'Ideal');

% 2. EKF estimated trajectory (blue solid line)
plot(ax_traj, state_data.estimate(:,1), state_data.estimate(:,3), 'b-', ...
    'LineWidth', 2, 'DisplayName', 'EKF Estimate');

% 3. Real trajectory (red solid line)
plot(ax_traj, state_data.real(:,1), state_data.real(:,3), 'r-', ...
    'LineWidth', 1.5, 'DisplayName', 'Real (Noisy)');

% 4. Measurement points (red dots)
scatter(ax_traj, state_data.real(1:30:end,1), output_data.real(1:30:end,1), ...
    25, 'r', 'filled', 'DisplayName', 'Measurements');

% Add pitch angle plates (every 20 time steps)
plate_interval = max(1, floor(N/20));
for i = 1:plate_interval:N
    x_pos = state_data.estimate(i, 1);
    y_pos = state_data.estimate(i, 3);
    theta = state_data.estimate(i, 5);
    
    % Create a short line showing pitch orientation
    line_length = 0.25;
    x_line = [x_pos, x_pos + line_length*cos(theta)];
    y_line = [y_pos, y_pos + line_length*sin(theta)];
    
    plot(ax_traj, x_line, y_line, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
end

% Start and end markers
scatter(ax_traj, state_data.clean(1,1), state_data.clean(1,3), ...
    80, 'g', 'filled', '^', 'DisplayName', 'Start');
scatter(ax_traj, state_data.clean(end,1), state_data.clean(end,3), ...
    80, 'm', 'filled', 'v', 'DisplayName', 'End');

xlabel(ax_traj, 'x Position (m)', 'FontWeight', 'bold');
ylabel(ax_traj, 'y Position (m)', 'FontWeight', 'bold');
title(ax_traj, '2D Trajectory with Pitch Angle Visualization', 'FontWeight', 'bold');
legend(ax_traj, 'Location', 'best', 'FontSize', 9);

%% ========== BOTTOM: ALL 6 STATES (2x3 grid) ==========
state_info = {
    {'x (m)', 'x Position'}, ...
    {'dx/dt (m/s)', 'x Velocity'}, ...
    {'y (m)', 'y Position'}, ...
    {'dy/dt (m/s)', 'y Velocity'}, ...
    {'θ (deg)', 'Pitch Angle'}, ...
    {'dθ/dt (deg/s)', 'Pitch Rate'} ...
};

% Subplot positions in 4x3 grid: [7, 8, 9, 10, 11, 12]
subplot_positions = [7, 8, 9, 10, 11, 12];

for i = 1:6
    ax_state = subplot(4, 3, subplot_positions(i));
    hold(ax_state, 'on'); grid(ax_state, 'on'); box(ax_state, 'on');
    
    % Convert angles to degrees for display
    if i == 5 || i == 6
        clean_data = rad2deg(state_data.clean(:,i));
        real_data = rad2deg(state_data.real(:,i));
        est_data = rad2deg(state_data.estimate(:,i));
    else
        clean_data = state_data.clean(:,i);
        real_data = state_data.real(:,i);
        est_data = state_data.estimate(:,i);
    end
    
    % Plot with required styling:
    % 1. Clean states: subtle (light gray dots, every 10th point)
    plot(ax_state, time(1:10:end), clean_data(1:10:end), '.', ...
        'Color', [0.7 0.7 0.7], 'MarkerSize', 6, 'DisplayName', 'Ideal');
    
    % 2. Real states: solid red line
    plot(ax_state, time, real_data, 'r-', ...
        'LineWidth', 1.5, 'DisplayName', 'Real State');
    
    % 3. Estimated states: solid blue line
    plot(ax_state, time, est_data, 'b-', ...
        'LineWidth', 2, 'DisplayName', 'EKF Estimate');
    
    % 4. Real outputs: red dots (only for measured states)
    if i == 3  % y position
        plot(ax_state, time(1:20:end), output_data.real(1:20:end,1), 'r.', ...
            'MarkerSize', 10, 'DisplayName', 'Real Output');
    elseif i == 5  % theta
        plot(ax_state, time(1:20:end), rad2deg(output_data.real(1:20:end,2)), 'r.', ...
            'MarkerSize', 10, 'DisplayName', 'Real Output');
    elseif i == 6  % theta-dot
        plot(ax_state, time(1:20:end), rad2deg(output_data.real(1:20:end,3)), 'r.', ...
            'MarkerSize', 10, 'DisplayName', 'Real Output');
    end
    
    xlabel(ax_state, 'Time (s)', 'FontSize', 9);
    ylabel(ax_state, state_info{i}{1}, 'FontSize', 9);
    title(ax_state, state_info{i}{2}, 'FontWeight', 'bold');
    
    % Add legend only to first state plot
    if i == 1
        legend(ax_state, 'Location', 'best', 'FontSize', 8);
    end
end

%% ========== RMSE DISPLAY (Top-right corner) ==========
ax_info = subplot(4, 3, 3);
axis(ax_info, 'off');

if exist('errors', 'var') && isfield(errors, 'rmse_states')
    rmse_text = {'PERFORMANCE SUMMARY:', '====================', ''};
    state_names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    
    for i = 1:min(6, length(errors.rmse_states))
        rmse_text{end+1} = sprintf('%s: %.4f', state_names{i}, errors.rmse_states(i));
    end
    
    % Add average error
    rmse_text{end+1} = '';
    rmse_text{end+1} = sprintf('Avg Pos Error: %.4f', mean(errors.rmse_states([1,3])));
    rmse_text{end+1} = sprintf('Avg Vel Error: %.4f', mean(errors.rmse_states([2,4])));
    rmse_text{end+1} = sprintf('Avg Ang Error: %.4f°', rad2deg(mean(errors.rmse_states([5,6]))));
else
    rmse_text = {'No RMSE data available'};
end

text(0.05, 0.95, rmse_text, ...
    'FontName', 'Courier New', 'FontSize', 9, ...
    'VerticalAlignment', 'top', 'FontWeight', 'bold', ...
    'BackgroundColor', [0.95 0.95 0.95]);

%% ========== OVERALL TITLE ==========
sgtitle('Planar Quadrotor: Complete State Estimation Results', ...
    'FontSize', 14, 'FontWeight', 'bold');

drawnow;
end