function plot_quadrotor_results(time, state, output, C, errors)
% plot_quadrotor_results - Visualize EKF performance for planar quadrotor
%
% Inputs:
%   time       : [N x 1] time vector (seconds)
%   state      : struct with fields .clean, .real, .estimate (each N x 6)
%   output     : struct with field .real (N x 3) = [y, theta, theta-dot] (measured)
%   C          : Measurement matrix (3x6)
%   errors     : (for future use)
%
% Outputs two figures:
%   1. Measured states: y, theta, theta-dot → clean vs noisy vs EKF
%   2. Full state vector: all 6 states → clean vs EKF
%
%

% Recompute clean and filtered outputs for consistency

output_clean = (C * state.clean')';
output_filtered = (C * state.estimate')';

% Ensure column vector
time = time(:);

%% FIGURE 1: Measured States — Perfect Conditions, Noisy, EKF
figure('Name', 'EKF Performance: Measured States', 'NumberTitle', 'off');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% y (altitude)
nexttile;
plot(time, output_clean(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(time, output.real(:,1), 'r.', 'MarkerSize', 8);
plot(time, output_filtered(:,1), 'g-', 'LineWidth', 1.5);
ylabel('y (m)'); grid on;
legend('Perfect Conditions', 'Noisy Measurement', 'EKF Estimate', 'Location', 'best');

% theta (pitch angle)
nexttile;
plot(time, rad2deg(output_clean(:,2)), 'b-', 'LineWidth', 1.5); hold on;
plot(time, rad2deg(output.real(:,2)), 'r.', 'MarkerSize', 8);
plot(time, rad2deg(output_filtered(:,2)), 'g-', 'LineWidth', 1.5);
ylabel('theta (deg)'); grid on;

% theta-dot (angular velocity)
nexttile;
plot(time, rad2deg(output_clean(:,3)), 'b-', 'LineWidth', 1.5); hold on;
plot(time, rad2deg(output.real(:,3)), 'r.', 'MarkerSize', 8);
plot(time, rad2deg(output_filtered(:,3)), 'g-', 'LineWidth', 1.5);
ylabel('theta-dot (deg/s)'); xlabel('Time (s)'); grid on;

sgtitle('Measured States: Perfect Conditions vs Noisy vs EKF Estimate');

%% FIGURE 2: Full State — Clean vs EKF Estimate
figure('Name', 'EKF Performance: Full State Estimation', 'NumberTitle', 'off');
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

state_labels = {'x (m)', 'x-dot (m/s)', 'y (m)', 'y-dot (m/s)', 'theta (deg)', 'theta-dot (deg/s)'};

for i = 1:6
    nexttile;
    if i <= 4
        plot(time, state.clean(:,i), 'b-', 'LineWidth', 1.5); hold on;
        plot(time, state.estimate(:,i), 'g-', 'LineWidth', 1.5);
        plot(time, state.real(:,i), 'r-', 'LineWidth', 1.5);
        ylabel(state_labels{i});
    else
        plot(time, rad2deg(state.clean(:,i)), 'b-', 'LineWidth', 1.5); hold on;
        plot(time, rad2deg(state.estimate(:,i)), 'g-', 'LineWidth', 1.5);
        plot(time, rad2deg(state.real(:,i)), 'r-', 'LineWidth', 1.5);
        ylabel(state_labels{i});
    end
    grid on;
    if i >= 5
        xlabel('Time (s)');
    end
end

sgtitle('Full State: Perfect Conditions vs EKF Estimate');
legend('Perfect Conditions', 'EKF Estimate', 'Real State', 'Location', 'southoutside', 'Orientation', 'horizontal');


end