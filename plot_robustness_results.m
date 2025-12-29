function plot_robustness_results(rmse_matrix, noise_matrix, divergence_data)
% PLOT_ROBUSTNESS_RESULTS - Visualize robustness analysis
% Takes pre-calculated data

% Force light theme
set(0, 'DefaultFigureColor', 'white');

% Get screen size
screen_size = get(0, 'ScreenSize');
fig_width = min(1000, screen_size(3) * 0.8);
fig_height = min(600, screen_size(4) * 0.8);

fig = figure('Name', 'Robustness Analysis', ...
    'NumberTitle', 'off', ...
    'Position', [(screen_size(3)-fig_width)/2, (screen_size(4)-fig_height)/2, fig_width, fig_height], ...
    'Color', 'white');

% 1. RMSE Bar Chart
subplot(2, 2, 1);
bar(rmse_matrix', 'grouped');
xlabel('State');
ylabel('RMSE');
title('RMSE Across Noise Conditions');
xticklabels({'x', 'dx', 'y', 'dy', 'θ', 'dθ'});
legend({'Optimal', 'Regular', 'High', 'Very High'}, 'Location', 'best');
grid on;

% 2. Noise Levels
subplot(2, 2, 2);
bar(noise_matrix);
xlabel('Case');
ylabel('Noise Amplitude');
title('Noise Levels');
xticklabels({'Optimal', 'Regular', 'High', 'Very High'});
legend({'Process', 'Measurement'}, 'Location', 'best');
grid on;

% 3. Divergence Analysis
subplot(2, 2, 3);
if ~isempty(divergence_data.multipliers)
    plot(divergence_data.multipliers, divergence_data.rmse_values(:,1), 'b-o', 'DisplayName', 'x');
    hold on; grid on;
    plot(divergence_data.multipliers, divergence_data.rmse_values(:,3), 'r-s', 'DisplayName', 'y');
    plot(divergence_data.multipliers, divergence_data.rmse_values(:,5), 'g-^', 'DisplayName', 'θ');
    
    if divergence_data.diverged
        xline(divergence_data.divergence_point, 'k--', 'LineWidth', 2, ...
            'DisplayName', sprintf('Divergence (%dx)', divergence_data.divergence_point));
    end
    
    xlabel('Noise Multiplier');
    ylabel('RMSE');
    title('RMSE Growth with Noise');
    legend('Location', 'best');
else
    text(0.5, 0.5, 'No divergence data', 'HorizontalAlignment', 'center');
    axis off;
end

% 4. Summary
subplot(2, 2, 4);
axis off;

summary = {
    'SUMMARY:';
    '========';
    sprintf('Avg RMSE (Optimal): %.4f', mean(rmse_matrix(1,:)));
    sprintf('Avg RMSE (Regular): %.4f', mean(rmse_matrix(2,:)));
    '';
    'DIVERGENCE:';
};

if divergence_data.diverged
    summary = [summary; {
        sprintf('At %dx normal noise', divergence_data.divergence_point);
        sprintf('State noise: %.4f', divergence_data.divergence_noise(1));
        sprintf('Output noise: %.4f', divergence_data.divergence_noise(2));
    }];
else
    summary = [summary; {
        'No divergence detected';
        'up to 50x noise';
    }];
end

text(0.05, 0.95, summary, 'FontName', 'Courier', 'FontSize', 9, 'VerticalAlignment', 'top');

sgtitle('EKF Robustness Analysis', 'FontSize', 12, 'FontWeight', 'bold');
end