function plot_quadrotor_enhanced(time, state, output, C, errors)

% Ensure time is column vector
time = time(:);
N = length(time);

% Recompute outputs for consistency
output_clean = (C * state.clean')';
output_filtered = (C * state.estimate')';


fig = figure('Name', 'Quadrotor Simulation - Complete Overview', ...
    'NumberTitle', 'off', ...
    'Units', 'normalized', ...
    'Position', [0.02 0.02 0.96 0.92], ...
    'Color', [1 1 1]);

%% ========== SUBPLOT 1: 2D TRAJECTORY WITH PITCH PLATES (TOP-LEFT, 2x2) ==========
subplot(3, 3, [1, 2, 4, 5]);
hold on; grid on; box on;
axis equal;

x_data = [state.clean(:,1); state.real(:,1); state.estimate(:,1)];
y_data = [state.clean(:,3); state.real(:,3); state.estimate(:,3)];

% Calculate min and max manually
x_min = min(x_data);
x_max = max(x_data);
y_min = min(y_data);
y_max = max(y_data);

x_mid = (x_min + x_max) / 2;
y_mid = (y_min + y_max) / 2;
x_range = x_max - x_min;
y_range = y_max - y_min;
max_range = max(x_range, y_range);

if max_range < 0.5
    max_range = 0.5; % Minimum scale for visibility
end
plot_margin = max_range * 0.2;


h_clean = plot(state.clean(:,1), state.clean(:,3), ...
    'Color', [0.85 0.85 0.85], 'LineStyle', ':', 'LineWidth', 0.5, ...
    'DisplayName', 'Clean Trajectory');


h_est = plot(state.estimate(:,1), state.estimate(:,3), ...
    'b-', 'LineWidth', 2, 'DisplayName', 'EKF Estimate');


h_real = plot(state.real(:,1), state.real(:,3), ...
    'r-', 'LineWidth', 1.2, 'DisplayName', 'Real States');


h_output = scatter(state.real(:,1), output.real(:,1), ...
    40, 'r', 'filled', 'Marker', 'o', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 0.5, ...
    'DisplayName', 'Real Output (y)');


plate_interval = max(1, floor(N/20)); 
plate_length = max_range * 0.15; 

for i = 1:plate_interval:N
    % Current position and orientation from ESTIMATE
    x_pos = state.estimate(i, 1);
    y_pos = state.estimate(i, 3);
    theta = state.estimate(i, 5); % Pitch angle in radians
    
   
    line_x = [-plate_length/2, plate_length/2];
    line_y = [0, 0];
    
    % Rotate line according to pitch angle
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated_line = R * [line_x; line_y];
    
    % Translate to current position
    line_x_final = rotated_line(1,:) + x_pos;
    line_y_final = rotated_line(2,:) + y_pos;
    
    % Plot the pitch plate as a thick black line
    plot(line_x_final, line_y_final, ...
        'k-', 'LineWidth', 3, 'HandleVisibility', 'off');
    
    % Add a small black circle at the center
    plot(x_pos, y_pos, 'ko', 'MarkerSize', 4, ...
        'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
end

% --- Add start and end markers ---
h_start = scatter(state.clean(1,1), state.clean(1,3), ...
    100, 'g', 'filled', '^', 'LineWidth', 1, ...
    'DisplayName', 'Start');
h_end = scatter(state.clean(end,1), state.clean(end,3), ...
    100, 'm', 'filled', 'v', 'LineWidth', 1, ...
    'DisplayName', 'End');

% --- Labels and formatting ---
xlabel('x (m)', 'FontWeight', 'bold'); 
ylabel('y (m)', 'FontWeight', 'bold');
title('2D Trajectory with Pitch Plates', 'FontWeight', 'bold');
legend([h_clean, h_est, h_real, h_output, h_start, h_end], ...
    'Location', 'bestoutside');

% Set axis limits with margin
xlim([x_mid - max_range/2 - plot_margin, x_mid + max_range/2 + plot_margin]);
ylim([y_mid - max_range/2 - plot_margin, y_mid + max_range/2 + plot_margin]);

%% ========== SUBPLOTS 2-7:  6 STATES ==========
state_info = {
    {'x (m)', 'x'}, ...
    {'dx/dt (m/s)', 'x-dot'}, ...
    {'y (m)', 'y'}, ...
    {'dy/dt (m/s)', 'y-dot'}, ...
    {'θ (rad)', 'theta'}, ...
    {'dθ/dt (rad/s)', 'theta-dot'} ...
};

% Which states have corresponding outputs?
has_output = [false, false, true, false, true, true];
output_map = [0, 0, 1, 0, 2, 3]; % Index into output.real

% Subplot positions in 3x3 grid
% Positions: 3, 4, 5, 6, 7, 9
subplot_positions = [3, 4, 5, 6, 7, 9];

for state_idx = 1:6
    % Set subplot position
    subplot(3, 3, subplot_positions(state_idx));
    
    % Clear and set up
    cla; hold on; grid on; box on;
    
    % Get label info
    y_label = state_info{state_idx}{1};
    state_name = state_info{state_idx}{2};
    
    % --- PLOT CLEAN STATE ---
    clean_step = max(1, floor(N/100)); % Show ~100 points
    clean_idx = 1:clean_step:N;
    h_clean_state = plot(time(clean_idx), state.clean(clean_idx, state_idx), ...
        '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 6, ...
        'DisplayName', 'Clean State');
    
    % --- PLOT REAL STATE (SOLID RED LINE) ---
    h_real_state = plot(time, state.real(:, state_idx), ...
        'r-', 'LineWidth', 1.5, 'DisplayName', 'Real State');
    
    % --- PLOT ESTIMATED STATE (SOLID BLUE LINE) ---
    h_est_state = plot(time, state.estimate(:, state_idx), ...
        'b-', 'LineWidth', 2, 'DisplayName', 'EKF Estimate');
    
    % --- PLOT REAL OUTPUT ---
    if has_output(state_idx) && output_map(state_idx) > 0
        output_col = output_map(state_idx);
        dot_step = max(1, floor(N/50)); % Show ~50 points
        dot_idx = 1:dot_step:N;
        h_output_state = plot(time(dot_idx), output.real(dot_idx, output_col), ...
            'r.', 'MarkerSize', 15, 'DisplayName', 'Real Output');
    end
    
    % --- Labels and formatting ---
    ylabel(y_label, 'FontWeight', 'bold');
    xlabel('Time (s)', 'FontWeight', 'bold');
    title(sprintf('State: %s', state_name), 'FontWeight', 'bold');
    
    % Set consistent time axis for ALL plots
    xlim([min(time), max(time)]);
    
    % Special handling for angle states
    if state_idx == 5 % theta (pitch angle)
        % Convert to degrees for readability
        ylabel('θ (deg)', 'FontWeight', 'bold');
        ylim_deg = [-180, 180]; % Reasonable range in degrees
        ylim(ylim_deg);
        
        % Convert and replot data in degrees
        cla; hold on; grid on; box on;
        
        % Clean state in degrees
        plot(time(clean_idx), rad2deg(state.clean(clean_idx, state_idx)), ...
            '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 6, ...
            'DisplayName', 'Clean State');
        
        % Real state in degrees
        plot(time, rad2deg(state.real(:, state_idx)), ...
            'r-', 'LineWidth', 1.5, 'DisplayName', 'Real State');
        
        % Estimated state in degrees
        plot(time, rad2deg(state.estimate(:, state_idx)), ...
            'b-', 'LineWidth', 2, 'DisplayName', 'EKF Estimate');
        
        % Output in degrees if applicable
        if has_output(state_idx) && output_map(state_idx) > 0
            plot(time(dot_idx), rad2deg(output.real(dot_idx, output_col)), ...
                'r.', 'MarkerSize', 15, 'DisplayName', 'Real Output');
        end
        
    elseif state_idx == 6 % theta-dot
        % Convert to deg/s for readability
        ylabel('dθ/dt (deg/s)', 'FontWeight', 'bold');
        
        % Replot in deg/s
        cla; hold on; grid on; box on;
        
        plot(time(clean_idx), rad2deg(state.clean(clean_idx, state_idx)), ...
            '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 6, ...
            'DisplayName', 'Clean State');
        plot(time, rad2deg(state.real(:, state_idx)), ...
            'r-', 'LineWidth', 1.5, 'DisplayName', 'Real State');
        plot(time, rad2deg(state.estimate(:, state_idx)), ...
            'b-', 'LineWidth', 2, 'DisplayName', 'EKF Estimate');
        
        if has_output(state_idx) && output_map(state_idx) > 0
            plot(time(dot_idx), rad2deg(output.real(dot_idx, output_col)), ...
                'r.', 'MarkerSize', 15, 'DisplayName', 'Real Output');
        end
    end
    
    % Add legend only to first state plot
    if state_idx == 1
        legend('Location', 'best', 'FontSize', 9);
    end
    
    % Ensure text fits in plot
    set(gca, 'FontSize', 9, 'FontWeight', 'bold');
end

%% ========== OVERALL FORMATTING ==========
% Add overall title
sgtitle('Planar Quadrotor: Complete State Estimation & Output Comparison', ...
    'FontSize', 16, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.5]);

% Adjust spacing
h = findobj(gcf, 'Type', 'axes');
for i = 1:length(h)
    set(h(i), 'FontSize', 9, 'FontWeight', 'bold');
end

drawnow;

%% ========== OPTIONAL: DISPLAY RMSE IN FIGURE ==========
if exist('errors', 'var') && isfield(errors, 'rmse_states')
    % Create a text box with RMSE values
    rmse_text = sprintf('RMSE Values:\n');
    state_names = {'x', 'x-dot', 'y', 'y-dot', 'theta', 'theta-dot'};
    
    for i = 1:min(6, length(errors.rmse_states))
        if i <= length(state_names)
            rmse_text = sprintf('%s%s: %.4f\n', ...
                rmse_text, state_names{i}, errors.rmse_states(i));
        end
    end
    
    % Add text annotation
    annotation('textbox', [0.02, 0.02, 0.2, 0.1], ...
        'String', rmse_text, ...
        'FontSize', 9, ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.95 0.95 0.95], ...
        'EdgeColor', [0.5 0.5 0.5]);
end

fprintf('Enhanced plot generated successfully!\n');
end