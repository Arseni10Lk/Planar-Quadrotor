function visualize_quadrotor_essential(time, state, output, C, errors)
% VISUALIZE_QUADROTOR_ESSENTIAL - Simple 2-window visualization
% Window 1: 3D Trajectory
% Window 2: State Dashboard

disp('Launching 2 essential visualization windows...');

% Window 1: Simple 3D Trajectory
figure('Name', '1. 3D Trajectory', 'NumberTitle', 'off');
ax1 = subplot(1,1,1);
hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
view(ax1, 3);

% Plot trajectories
plot3(ax1, state.clean(:,1), state.clean(:,3), zeros(length(time),1), 'k:', 'LineWidth', 1);
plot3(ax1, state.estimate(:,1), state.estimate(:,3), zeros(length(time),1), 'b-', 'LineWidth', 2);
plot3(ax1, state.real(:,1), state.real(:,3), zeros(length(time),1), 'r-', 'LineWidth', 1);

% Add start/end markers
scatter3(ax1, state.clean(1,1), state.clean(1,3), 0.1, 100, 'g', 'filled', '^');
scatter3(ax1, state.clean(end,1), state.clean(end,3), 0.1, 100, 'r', 'filled', 'v');

% Labels
xlabel(ax1, 'X Position (m)');
ylabel(ax1, 'Y Position (m)');
zlabel(ax1, 'Z (m)');
title(ax1, '3D Quadrotor Trajectory');
legend(ax1, {'Ideal', 'EKF Estimate', 'Real', 'Start', 'End'}, 'Location', 'best');
axis(ax1, 'equal');

% Window 2: State Dashboard
figure('Name', '2. State Dashboard', 'NumberTitle', 'off');

% Plot all 6 states
for i = 1:6
    subplot(2,3,i);
    hold on; grid on; box on;
    
    % Convert angles to degrees for display
    if i == 5 || i == 6
        clean_data = rad2deg(state.clean(:,i));
        real_data = rad2deg(state.real(:,i));
        est_data = rad2deg(state.estimate(:,i));
        if i == 5
            ylabel_str = 'Pitch (deg)';
        else
            ylabel_str = 'Pitch Rate (deg/s)';
        end
    else
        clean_data = state.clean(:,i);
        real_data = state.real(:,i);
        est_data = state.estimate(:,i);
        if i == 1
            ylabel_str = 'x (m)';
        elseif i == 2
            ylabel_str = 'dx/dt (m/s)';
        elseif i == 3
            ylabel_str = 'y (m)';
        elseif i == 4
            ylabel_str = 'dy/dt (m/s)';
        end
    end
    
    % Plot data
    plot(time, clean_data, 'k:', 'LineWidth', 0.5);
    plot(time, real_data, 'r-', 'LineWidth', 1);
    plot(time, est_data, 'b-', 'LineWidth', 1.5);
    
    % Add measurements for y, theta, theta-dot
    if i == 3
        scatter(time(1:20:end), output.real(1:20:end,1), 20, 'r', 'filled');
    elseif i == 5
        scatter(time(1:20:end), rad2deg(output.real(1:20:end,2)), 20, 'r', 'filled');
    elseif i == 6
        scatter(time(1:20:end), rad2deg(output.real(1:20:end,3)), 20, 'r', 'filled');
    end
    
    xlabel('Time (s)');
    ylabel(ylabel_str);
    
    if i == 1
        title('X Position');
    elseif i == 2
        title('X Velocity');
    elseif i == 3
        title('Y Position');
    elseif i == 4
        title('Y Velocity');
    elseif i == 5
        title('Pitch Angle');
    elseif i == 6
        title('Pitch Rate');
    end
    
    if i == 1
        legend({'Ideal', 'Real', 'EKF', 'Measurements'}, 'Location', 'best', 'FontSize', 8);
    end
end

disp('Both windows created successfully!');
end