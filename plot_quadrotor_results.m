function plot_quadrotor_results(time, state, output, C)
% plot_quadrotor_results - Visualize EKF performance for planar quadrotor
%
% Inputs:
%   time       : [N x 1] time vector (seconds)
%   state      : struct with fields .clean, .real, .estimate (each N x 6)
%   output     : struct with field .real (N x 3) = [y_meas, theta_meas, thetadot_meas]
%   C          : Measurement matrix (3x6)
%
% Generates two figures:
%   1. Measured states: y, theta, theta-dot → clean vs noisy vs EKF
%   2. Full state vector: x, xdot, y, ydot, theta, thetadot → clean vs EKF

% Recompute clean and filtered outputs for consistency
output_clean = (C * state.clean')';
output_filtered = (C * state.estimate')';

% Ensure time is a column vector
time = time(:);

%% FIGURE 1: Measured States — Clean, Noisy, Filtered
figure('Name', 'EKF Performance: Measured States', 'NumberTitle', 'off');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% y position (m)
nexttile;
plot(time, output_clean(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(time, output.real(:,1), 'r.', 'MarkerSize', 8);
plot(time, output_filtered(:,1), 'g--', 'LineWidth', 1.5);
ylabel('y (m)'); grid on;
legend('Ground Truth', 'Noisy Measurement', 'EKF Estimate', 'Location', 'best');

% Theta (degrees)
nexttile;
plot(time, rad2deg(output_clean(:,2)), 'b-', 'LineWidth', 1.5); hold on;
plot(time, rad2deg(output.real(:,2)), 'r.', 'MarkerSize', 8);
plot(time, rad2deg(output_filtered(:,2)), 'g--', 'LineWidth', 1.5);
ylabel('theta (deg)'); grid on;

% Theta-dot (deg/s)
nexttile;
plot(time, rad2deg(output_clean(:,3)), 'b-', 'LineWidth', 1.5); hold on;
plot(time, rad2deg(output.real(:,3)), 'r.', 'MarkerSize', 8);
plot(time, rad2deg(output_filtered(:,3)), 'g--', 'LineWidth', 1.5);
ylabel('theta-dot (deg/s)'); xlabel('Time (s)'); grid on;

sgtitle('Measured States: Ground Truth vs Noisy vs EKF Estimate');

%% FIGURE 2: Full State — Clean vs EKF Estimate
figure('Name', 'EKF Performance: Full State Estimation', 'NumberTitle', 'off');
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% State variable names for y-labels
state_names = {'x (m)', 'x-dot (m/s)', 'y (m)', 'y-dot (m/s)', 'theta (deg)', 'theta-dot (deg/s)'};

for i = 1:6
    nexttile;
    if i <= 4
        % Linear states: x, xdot, y, ydot (in m or m/s)
        plot(time, state.clean(:,i), 'b-', 'LineWidth', 1.5); hold on;
        plot(time, state.estimate(:,i), 'g--', 'LineWidth', 1.5);
        ylabel(state_names{i});
    else
        % Angular states: convert to degrees
        plot(time, rad2deg(state.clean(:,i)), 'b-', 'LineWidth', 1.5); hold on;
        plot(time, rad2deg(state.estimate(:,i)), 'g--', 'LineWidth', 1.5);
        ylabel(state_names{i});
    end
    grid on;
    if i >= 5
        xlabel('Time (s)');
    end
end

sgtitle('Full State: Ground Truth vs EKF Estimate');
legend('Ground Truth', 'EKF Estimate', 'Location', 'southoutside', 'Orientation', 'horizontal');