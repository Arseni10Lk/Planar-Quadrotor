function plot_quadrotor_enhanced(time, state_data, output_data, C, errors)
% plot_quadrotor_enhanced - Enhanced visualization for planar quadrotor EKF results
%
% Inputs:
%   time        : [N x 1] time vector (seconds)
%   state_data  : struct with fields .clean, .real, .estimate (each N x 6)
%   output_data : struct with field .real (N x 3) = [y, theta, theta-dot]
%   C           : Measurement matrix (3x6)
%   errors      : struct with RMSE and other error metrics
%


% Force light theme for consistent display
set(0, 'DefaultFigureColor', 'white');

% Get screen size for optimal window placement
screen_size = get(0, 'ScreenSize');
fig_width = min(1200, screen_size(3) * 0.85);
fig_height = min(900, screen_size(4) * 0.85);
fig_x = (screen_size(3) - fig_width) / 2;
fig_y = (screen_size(4) - fig_height) / 2;

% Create single figure window
fig = figure('Name', 'Quadrotor Simulation - Complete Results', ...
             'NumberTitle', 'off', ...
             'Position', [fig_x, fig_y, fig_width, fig_height], ...
             'Color', 'white');

%% ================= TOP-LEFT: 2D TRAJECTORY WITH THRUST ARROWS =================
ax_traj = subplot(4, 4, [1 2 5 6]);
hold(ax_traj, 'on'); 
grid(ax_traj, 'on'); 
box(ax_traj, 'on');
axis(ax_traj, 'equal');

N = length(time);

% 1. Ideal trajectory (gray dashed line)
plot(ax_traj, state_data.clean(:,1), state_data.clean(:,3), '--', ...
     'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'DisplayName', 'Ideal Trajectory');

% 2. EKF estimated trajectory (blue solid line)
plot(ax_traj, state_data.estimate(:,1), state_data.estimate(:,3), 'b-', ...
     'LineWidth', 2.5, 'DisplayName', 'EKF Estimate');

% 3. Real trajectory (red solid line)
plot(ax_traj, state_data.real(:,1), state_data.real(:,3), 'r-', ...
     'LineWidth', 1.5, 'DisplayName', 'Real (Noisy)');

% 4. Measurement points (magenta dots)
scatter(ax_traj, state_data.real(1:25:end,1), output_data.real(1:25:end,1), ...
        40, 'm', 'filled', '^', 'DisplayName', 'Measurements');

% 5. Add thrust direction arrows (every 15 time steps)
arrow_interval = max(1, floor(N/15));
for i = 1:arrow_interval:N
    x_pos = state_data.estimate(i, 1);
    y_pos = state_data.estimate(i, 3);
    theta = state_data.estimate(i, 5);
    
    % Arrow parameters
    arrow_length = 0.25;
    head_size = 0.08;
    
    % Calculate arrow direction (thrust direction is along body axis)
    dx = arrow_length * cos(theta);
    dy = arrow_length * sin(theta);
    
    % Draw arrow with fixed appearance
    if i == 1
        % First arrow with legend entry
        quiver(ax_traj, x_pos, y_pos, dx, dy, ...
               'MaxHeadSize', head_size/arrow_length, ...
               'Color', [0 0.5 0], 'LineWidth', 2, ...
               'AutoScale', 'off', 'DisplayName', 'Thrust Direction');
    else
        % Subsequent arrows without legend
        quiver(ax_traj, x_pos, y_pos, dx, dy, ...
               'MaxHeadSize', head_size/arrow_length, ...
               'Color', [0 0.5 0], 'LineWidth', 1.5, ...
               'AutoScale', 'off', 'HandleVisibility', 'off');
    end
    
    % Add small dot at arrow base
    scatter(ax_traj, x_pos, y_pos, 20, [0 0.5 0], 'filled', ...
            'HandleVisibility', 'off');
end

% 6. Start and end markers
scatter(ax_traj, state_data.clean(1,1), state_data.clean(1,3), ...
        100, 'g', 'filled', '^', 'LineWidth', 2, ...
        'DisplayName', 'Start');
scatter(ax_traj, state_data.clean(end,1), state_data.clean(end,3), ...
        100, [1 0.5 0], 'filled', 'v', 'LineWidth', 2, ...
        'DisplayName', 'End');

xlabel(ax_traj, 'x Position (m)', 'FontWeight', 'bold', 'FontSize', 10);
ylabel(ax_traj, 'y Position (m)', 'FontWeight', 'bold', 'FontSize', 10);
title(ax_traj, '2D Trajectory with Thrust Direction', ...
      'FontWeight', 'bold', 'FontSize', 11);
legend(ax_traj, 'Location', 'best', 'FontSize', 9);

%% ================= TOP-RIGHT: RMSE SUMMARY ======================
ax_info = subplot(4, 4, [3 4]);
axis(ax_info, 'off');

if exist('errors', 'var') && isfield(errors, 'rmse_states')
    % Format RMSE display with proper units
    rmse_text = {'PERFORMANCE SUMMARY', ...
                 '=====================', ...
                 ''};
    
    state_names = {'x pos (m)     ', 'x vel (m/s)   ', ...
                   'y pos (m)     ', 'y vel (m/s)   ', ...
                   'θ (deg)       ', 'dθ (deg/s)    '};
    
    % Display RMSE for each state
    for i = 1:min(6, length(errors.rmse_states))
        % Convert angles to degrees for display
        if i == 5 || i == 6
            value = rad2deg(errors.rmse_states(i));
            unit = '';
        else
            value = errors.rmse_states(i);
            unit = ' m';
            if i == 2 || i == 4
                unit = ' m/s';
            end
        end
        rmse_text{end+1} = sprintf('%s: %6.4f%s', state_names{i}, value, unit);
    end
    
    rmse_text{end+1} = '';
    rmse_text{end+1} = '------------------------';
    rmse_text{end+1} = sprintf('Avg Pos Error: %6.4f m', ...
                               mean(errors.rmse_states([1,3])));
    rmse_text{end+1} = sprintf('Avg Vel Error: %6.4f m/s', ...
                               mean(errors.rmse_states([2,4])));
    rmse_text{end+1} = sprintf('Avg Ang Error: %6.4f°', ...
                               rad2deg(mean(errors.rmse_states([5,6]))));
    
    % Add overall performance rating
    avg_rmse = mean(errors.rmse_states(1:4)); % Average of position/velocity
    if avg_rmse < 0.01
        rating = 'EXCELLENT';
        color = [0 0.6 0];
    elseif avg_rmse < 0.05
        rating = 'GOOD';
        color = [0 0.4 0.8];
    elseif avg_rmse < 0.1
        rating = 'FAIR';
        color = [1 0.5 0];
    else
        rating = 'POOR';
        color = [0.8 0 0];
    end
    
    rmse_text{end+1} = '';
    rmse_text{end+1} = 'Overall Rating:';
    rmse_text{end+1} = sprintf('  %s', rating);
    
else
    rmse_text = {'No RMSE data available'};
    color = [0.3 0.3 0.3];
end

% Display text with colored rating
text(0.05, 0.95, rmse_text, ...
     'FontName', 'FixedWidth', 'FontSize', 9, ...
     'VerticalAlignment', 'top', ...
     'BackgroundColor', [0.97 0.97 0.97], ...
     'EdgeColor', [0.8 0.8 0.8], 'LineWidth', 1);

% Highlight rating in color
if exist('errors', 'var') && isfield(errors, 'rmse_states')
    rating_pos = find(strcmp(rmse_text, sprintf('  %s', rating)));
    if ~isempty(rating_pos)
        text_handles = findobj(ax_info, 'Type', 'text');
        if length(text_handles) >= rating_pos
            set(text_handles(rating_pos), 'Color', color, 'FontWeight', 'bold');
        end
    end
end

title(ax_info, 'State Estimation Accuracy', 'FontWeight', 'bold', 'FontSize', 11);

%% ================= BOTTOM: ALL 6 STATES (2x3 grid) =================
% Subplot positions in 4x4 grid: [9, 10, 11, 12, 13, 14]
subplot_positions = [9, 10, 11, 12, 13, 14];

state_info = {
    {'x (m)', 'Position (x)'};
    {'dx/dt (m/s)', 'Velocity (x)'};
    {'y (m)', 'Position (y)'};
    {'dy/dt (m/s)', 'Velocity (y)'};
    {'θ (deg)', 'Pitch Angle'};
    {'dθ/dt (deg/s)', 'Pitch Rate'};
};

% Colors and markers for different data types
colors = struct(...
    'ideal', [0.6 0.6 0.6], ...
    'real', [0.8 0.2 0.2], ...
    'estimate', [0.2 0.4 0.8], ...
    'measurement', [0.8 0.2 0.8] ...
);

for i = 1:6
    ax_state = subplot(4, 4, subplot_positions(i));
    hold(ax_state, 'on'); 
    grid(ax_state, 'on'); 
    box(ax_state, 'on');
    
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
    
    % 1. Ideal states: light gray dots (every 10th point)
    plot(ax_state, time(1:10:end), clean_data(1:10:end), '.', ...
         'Color', colors.ideal, 'MarkerSize', 8, ...
         'DisplayName', 'Ideal');
    
    % 2. Real states: red line
    plot(ax_state, time, real_data, '-', ...
         'Color', colors.real, 'LineWidth', 1, ...
         'DisplayName', 'Real State');
    
    % 3. Estimated states: blue line
    plot(ax_state, time, est_data, '-', ...
         'Color', colors.estimate, 'LineWidth', 2, ...
         'DisplayName', 'EKF Estimate');
    
    % 4. Measurements: magenta dots (only for measured states)
    is_measured = false;
    if i == 3 % y position is measured
        plot(ax_state, time(1:20:end), output_data.real(1:20:end,1), '.', ...
             'Color', colors.measurement, 'MarkerSize', 12, ...
             'DisplayName', 'Measurement');
        is_measured = true;
    elseif i == 5 % theta is measured
        plot(ax_state, time(1:20:end), rad2deg(output_data.real(1:20:end,2)), '.', ...
             'Color', colors.measurement, 'MarkerSize', 12, ...
             'DisplayName', 'Measurement');
        is_measured = true;
    elseif i == 6 % theta-dot is measured
        plot(ax_state, time(1:20:end), rad2deg(output_data.real(1:20:end,3)), '.', ...
             'Color', colors.measurement, 'MarkerSize', 12, ...
             'DisplayName', 'Measurement');
        is_measured = true;
    end
    
    % Set labels and title
    xlabel(ax_state, 'Time (s)', 'FontSize', 9);
    ylabel(ax_state, state_info{i,1}{1}, 'FontSize', 9);
    title(ax_state, state_info{i,1}{2}, 'FontWeight', 'bold', 'FontSize', 10);
    
    % Add legend to first plot only
    if i == 1
        if is_measured
            legend(ax_state, 'Location', 'best', 'FontSize', 8, 'NumColumns', 2);
        else
            legend(ax_state, 'Location', 'best', 'FontSize', 8);
        end
    end
    
    % Add grid with subtle style
    set(ax_state, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
end

%% ================= ERROR PLOT (Bottom-right) ======================
ax_error = subplot(4, 4, [15 16]);
hold(ax_error, 'on'); 
grid(ax_error, 'on'); 
box(ax_error, 'on');

% Calculate estimation errors for each state
if exist('errors', 'var') && isfield(errors, 'states_real_VS_estimate')
    error_data = errors.states_real_VS_estimate;
    
    % Convert angular errors to degrees
    error_data(:,5) = rad2deg(error_data(:,5));
    error_data(:,6) = rad2deg(error_data(:,6));
    
    % Plot error bounds (95% confidence interval)
    time_smooth = linspace(time(1), time(end), 100);
    for i = 1:6
        % Smooth errors using moving average
        window_size = min(50, length(time));
        smooth_error = movmean(abs(error_data(:,i)), window_size);
        
        % Resample for smooth plotting
        smooth_error_interp = interp1(time, smooth_error, time_smooth, 'pchip');
        
        % Plot smoothed absolute error
        plot(ax_error, time_smooth, smooth_error_interp, ...
             'LineWidth', 1.5, 'DisplayName', state_info{i,1}{1});
    end
    
    xlabel(ax_error, 'Time (s)', 'FontSize', 9);
    ylabel(ax_error, 'Absolute Error', 'FontSize', 9);
    title(ax_error, 'State Estimation Errors (Smoothed)', ...
          'FontWeight', 'bold', 'FontSize', 10);
    legend(ax_error, 'Location', 'best', 'FontSize', 8, 'NumColumns', 2);
    
    % Add horizontal line at zero for reference
    y_limits = ylim(ax_error);
    plot(ax_error, [time(1) time(end)], [0 0], 'k--', ...
         'LineWidth', 0.5, 'HandleVisibility', 'off');
    
else
    text(ax_error, 0.5, 0.5, 'No error data available', ...
         'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle');
    axis(ax_error, 'off');
end

%% ================= OVERALL TITLE ======================
sgtitle({'Planar Quadrotor: Extended Kalman Filter State Estimation', ...
         sprintf('Simulation Duration: %.1f seconds | Time Step: %.3f s', ...
         time(end), time(2)-time(1))}, ...
        'FontSize', 14, 'FontWeight', 'bold');

% Add timestamp
annotation('textbox', [0.02, 0.02, 0.2, 0.03], ...
           'String', sprintf('Generated: %s', datestr(now)), ...
           'FontSize', 8, 'EdgeColor', 'none', ...
           'HorizontalAlignment', 'left');

% Ensure everything is drawn
drawnow;

% Print summary to console
if exist('errors', 'var') && isfield(errors, 'rmse_states')
    fprintf('\n=== QUADROTOR EKF PERFORMANCE SUMMARY ===\n');
    fprintf('Average Position RMSE: %.4f m\n', mean(errors.rmse_states([1,3])));
    fprintf('Average Velocity RMSE: %.4f m/s\n', mean(errors.rmse_states([2,4])));
    fprintf('Average Angle RMSE: %.4f°\n', rad2deg(mean(errors.rmse_states([5,6]))));
    fprintf('=========================================\n');
end

end