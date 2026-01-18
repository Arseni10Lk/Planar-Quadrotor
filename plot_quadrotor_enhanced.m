function plot_quadrotor_enhanced(time, state_data, output_data, C, errors)

% Force light theme
set(0, 'DefaultFigureColor', 'white');

% Get screen size and calculate figure dimensions
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

%% ================= 1. CLEAN 2D TRAJECTORY (Top-Left 2x2) =================
% Spanning rows 1 and 2, columns 1 and 2
ax_traj = subplot(4, 4, [1 2 5 6]);
hold(ax_traj, 'on'); 
grid(ax_traj, 'on'); 
box(ax_traj, 'on');
axis(ax_traj, 'equal');

% === PLOT DATA (No DisplayNames needed here, legends are offloaded) ===
plot(ax_traj, state_data.clean(:,1), state_data.clean(:,3), '--', ...
     'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);

plot(ax_traj, state_data.estimate(:,1), state_data.estimate(:,3), 'b-', ...
     'LineWidth', 2.5);

plot(ax_traj, state_data.real(:,1), state_data.real(:,3), 'r-', ...
     'LineWidth', 1.5);

% === ADD ORIENTATION BARS ===
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

% Start/End Markers
scatter(ax_traj, state_data.real(1,1), state_data.real(1,3), ...
        50, [0 0.7 0.7], 'filled', '^', 'LineWidth', 1, 'MarkerEdgeColor', 'k');
scatter(ax_traj, state_data.real(end,1), state_data.real(end,3), ...
        50, [1 0.5 0], 'filled', 'v', 'LineWidth', 1, 'MarkerEdgeColor', 'k');

% Labels
xlabel(ax_traj, 'Horizontal Position, x (m)', 'FontWeight', 'bold', 'FontSize', 10);
ylabel(ax_traj, 'Altitude, y (m)', 'FontWeight', 'bold', 'FontSize', 10);
title(ax_traj, '2D Trajectory', 'FontWeight', 'bold', 'FontSize', 11);

% Fix Axis Limits to avoid crunching
x_margin = bar_length * 0.8;
y_margin = bar_length * 0.8;
xlim(ax_traj, [min(state_data.estimate(:,1))-x_margin, max(state_data.estimate(:,1))+x_margin]);
ylim(ax_traj, [min(state_data.estimate(:,3))-y_margin, max(state_data.estimate(:,3))+y_margin]);


%% ================= 2. RMSE SUMMARY & DUMMY LEGENDS (Top-Right 2x2) =================
% Spanning rows 1 & 2, cols 3 & 4.
ax_info = subplot(4, 4, [3 4 7 8]);
hold(ax_info, 'on');
axis(ax_info, 'off');
xlim(ax_info, [0 1]);
ylim(ax_info, [0 1]);

% --- A. DUMMY PLOTS FOR TRAJECTORY LEGEND ---
% Invisible lines used only to generate the legend entries on this panel
h_d_ideal = plot(ax_info, NaN, NaN, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'DisplayName', 'Ideal');
h_d_ekf   = plot(ax_info, NaN, NaN, 'b-', 'LineWidth', 2.5, 'DisplayName', 'EKF');
h_d_real  = plot(ax_info, NaN, NaN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Real');
h_d_body  = plot(ax_info, NaN, NaN, '-', 'Color', [0.1 0.6 0.1], 'LineWidth', 5, 'DisplayName', 'Body');
h_d_thrust= plot(ax_info, NaN, NaN, '-', 'Color', [0.9 0.1 0.1], 'LineWidth', 2, 'DisplayName', 'Thrust');
h_d_start = scatter(ax_info, NaN, NaN, 100, [0 0.7 0.7], 'filled', '^', 'MarkerEdgeColor', 'k', 'DisplayName', 'Start');
h_d_end   = scatter(ax_info, NaN, NaN, 100, [1 0.5 0], 'filled', 'v', 'MarkerEdgeColor', 'k', 'DisplayName', 'End');

% TRAJECTORY LEGEND -> PLACED INSIDE 'NORTH'
% 'North' keeps it inside the box, so it doesn't trigger layout resizing.
hL_traj = legend(ax_info, [h_d_ideal, h_d_ekf, h_d_real, h_d_body, h_d_thrust, h_d_start, h_d_end], ...
       'Location', 'north', 'Orientation', 'horizontal', 'FontSize', 8, 'NumColumns', 3);
set(hL_traj, 'ItemTokenSize', [15, 18], 'Box', 'off');
title(hL_traj, 'Trajectory Legend');

% --- B. RMSE TEXT ---
if exist('errors', 'var') && isfield(errors, 'rmse_states')
    rmse_text = {'ESTIMATION ACCURACY (RMSE)', '===============================', ''};
    
    state_labels = {
        'x (Pos) : '; 'dx(Vel) : ';
        'y (Pos) : '; 'dy(Vel) : ';
        'θ (Ang) : '; 'dθ(Rate): '
    };
    
    for i = 1:6
        if i <= 4
            val = errors.rmse_states(i);
            unit = ' m'; if mod(i,2)==0, unit=' m/s'; end
        else
            val = rad2deg(errors.rmse_states(i));
            unit = ' deg'; if i==6, unit=' deg/s'; end
        end
        rmse_text{end+1} = sprintf('%s%7.4f%s', state_labels{i}, val, unit);
    end
    
else
    rmse_text = {'No RMSE data'};
end

% Draw text in the MIDDLE (y=0.45) to avoid overlapping with top/bottom legends
text(0.5, 0.45, rmse_text, ...
     'Parent', ax_info, ...
     'FontName', 'FixedWidth', 'FontSize', 9, ...
     'HorizontalAlignment', 'center', ...
     'VerticalAlignment', 'middle', ...
     'BackgroundColor', [0.98 0.98 0.98], ...
     'EdgeColor', [0.9 0.9 0.9], 'Margin', 6);


%% ================= 3. ALL 6 STATE PLOTS (Rows 3 & 4) =================
state_positions = [9, 10, 11, 12, 13, 14];
state_config = {
    {'x (m)', 'Horizontal Position'}; {'dx/dt (m/s)', 'Horizontal Velocity'};
    {'y (m)', 'Vertical Position'};   {'dy/dt (m/s)', 'Vertical Velocity'};
    {'θ (deg)', 'Pitch Angle'};       {'dθ/dt (deg/s)', 'Pitch Rate'};
};

blue_lw = 2.0; red_lw = 3.0;
plot_handles = [];

for idx = 1:6
    ax_state = subplot(4, 4, state_positions(idx));
    hold(ax_state, 'on'); grid(ax_state, 'on'); box(ax_state, 'on');
    
    if idx >= 5, conv = @rad2deg; else, conv = @(x) x; end
    
    h1 = plot(ax_state, time(1:20:end), conv(state_data.clean(1:20:end, idx)), '.', ...
         'Color', [0.5 0.5 0.5], 'MarkerSize', 5, 'DisplayName', 'Ideal');
    h2 = plot(ax_state, time, conv(state_data.real(:, idx)), '-', ...
         'Color', [0.85 0.2 0.2], 'LineWidth', red_lw, 'DisplayName', 'Real');
    h3 = plot(ax_state, time, conv(state_data.estimate(:, idx)), '-', ...
         'Color', [0.2 0.3 0.9], 'LineWidth', blue_lw, 'DisplayName', 'EKF');
         
    if idx == 1, plot_handles = [h1, h2, h3]; end
    
    ylabel(ax_state, state_config{idx, 1}{1}, 'FontSize', 8);
    title(ax_state, state_config{idx, 1}{2}, 'FontWeight', 'bold', 'FontSize', 9);
    set(ax_state, 'GridAlpha', 0.3);
end

%% ================= 4. ERROR EVOLUTION PLOT (Bottom Right) =================
ax_error = subplot(4, 4, [15 16]);
hold(ax_error, 'on'); grid(ax_error, 'on'); box(ax_error, 'on');

if exist('errors', 'var') && isfield(errors, 'states_real_VS_estimate')
    % Data Prep
    err_k = abs(errors.states_real_VS_estimate);
    err_k(:,5:6) = rad2deg(err_k(:,5:6));
    err_r = abs(errors.states_real_VS_running);
    err_r(:,5:6) = rad2deg(err_r(:,5:6));
    
    colors = lines(6);
    names = {'x', 'dx', 'y', 'dy', 'θ', 'dθ'};
    max_y = 0;
    
    for i = 1:6
        w = min(40, floor(length(time)/8));
        
        % Plot Running (Dashed) - Plot FIRST so it appears in legend but stays behind if needed
        % Now using DisplayName for legend visibility
        plot(ax_error, time, movmean(err_r(:,i), w), '--', ...
             'Color', colors(i,:), 'LineWidth', 1.0, ...
             'DisplayName', [names{i} ' (Running Mean)']);
             
        % Plot EKF (Solid)
        ekf_line = movmean(err_k(:,i), w);
        max_y = max(max_y, max(ekf_line));
        plot(ax_error, time, ekf_line, '-', ...
             'Color', colors(i,:), 'LineWidth', 1.5, ...
             'DisplayName', [names{i} ' (EKF)']);
    end
    
    if max_y > 0, ylim(ax_error, [0, max_y * 1.3]); end
    xlabel(ax_error, 'Time (s)', 'FontSize', 9);
    ylabel(ax_error, 'Abs Error', 'FontSize', 9);
    title(ax_error, 'Estimation Errors', 'FontWeight', 'bold', 'FontSize', 10);
    
    % ERROR LEGEND -> CHANGE: Moved to 'eastoutside' to prevent overlap
    legend(ax_error, 'Location', 'eastoutside', 'FontSize', 8, 'Box', 'on');
else
    text(0.5, 0.5, 'No Data', 'Parent', ax_error, 'HorizontalAlignment', 'center');
    axis(ax_error, 'off');
end

sgtitle('Planar Quadrotor: Final State Estimation Results', 'FontSize', 14, 'FontWeight', 'bold');
end