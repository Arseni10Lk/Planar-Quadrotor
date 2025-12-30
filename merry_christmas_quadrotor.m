function merry_christmas_quadrotor()
% MERRY_CHRISTMAS_QUADROTOR - Festive Christmas display with quadrotor theme
% Fixed version without brighten function errors

% Create festive figure
fig = figure('Name', 'Merry Christmas!', 'NumberTitle', 'off', ...
    'Position', [100 100 1000 800], 'Color', [0.05 0.05 0.1], ...
    'MenuBar', 'none', 'ToolBar', 'none');

% Main axes
ax = axes('Parent', fig, 'Position', [0.05 0.05 0.9 0.9]);
hold(ax, 'on'); axis(ax, 'equal'); axis(ax, 'off');

% Set limits
xlim(ax, [-15 15]);
ylim(ax, [-5 25]);

%% ========== CREATE STAR BACKGROUND ==========
% Create twinkling stars
n_stars = 100;
star_x = 30 * rand(1, n_stars) - 15;
star_y = 25 * rand(1, n_stars);
star_size = 5 + 10 * rand(1, n_stars);

% Plot stars with twinkling effect
stars = scatter(ax, star_x, star_y, star_size, 'w', 'filled', 'Marker', 'p');
set(stars, 'MarkerFaceAlpha', 0.7);

%% ========== CREATE CHRISTMAS TREE ==========
% Tree trunk
fill(ax, [-0.5 0.5 0.5 -0.5], [0 0 2 2], [0.4 0.2 0], 'EdgeColor', 'none');

% Tree layers (from bottom to top)
tree_colors = [0.1 0.5 0.1; 0.2 0.6 0.2; 0.3 0.7 0.3; 0.4 0.8 0.4];
layer_heights = [2, 5, 8, 11];
layer_widths = [6, 5, 4, 3];

for i = 1:4
    x = [-layer_widths(i)/2, 0, layer_widths(i)/2];
    y = [layer_heights(i), layer_heights(i)+3, layer_heights(i)];
    fill(ax, x, y, tree_colors(i,:), 'EdgeColor', [0.2 0.4 0.2], 'LineWidth', 2);
end

% Tree topper (star)
star_angles = 0:72:360;
star_x_top = 0.5 * cosd(star_angles + 18);
star_y_top = 0.5 * sind(star_angles + 18) + 14;
fill(ax, star_x_top, star_y_top, [1 0.8 0], 'EdgeColor', [1 0.9 0], 'LineWidth', 2);

% Add ornaments (baubles) - store handles for animation
ornament_handles = [];
ornament_positions = [
    -2.5, 3; 2.5, 3;
    -1.8, 5; 1.8, 5;
    -1.2, 7; 1.2, 7;
    -0.6, 9; 0.6, 9;
    0, 11;
];
ornament_colors = {'r', 'b', 'y', 'm', 'c', 'g', 'w', 'r', 'b', 'y'};

for i = 1:size(ornament_positions, 1)
    color = ornament_colors{i};
    h = plot(ax, ornament_positions(i,1), ornament_positions(i,2), ...
        'o', 'MarkerSize', 12, 'MarkerFaceColor', color, ...
        'MarkerEdgeColor', 'w', 'LineWidth', 2);
    ornament_handles = [ornament_handles, h];
end

% Add tinsel/garland
theta = linspace(0, 4*pi, 100);
tinsel_x = 0.8 * sin(theta) .* (1 - 0.1 * theta);
tinsel_y = 2 + theta * 0.8;
plot(ax, tinsel_x, tinsel_y, '-', 'Color', [1 0.5 0], 'LineWidth', 1.5);

%% ========== CREATE SANTA QUADROTOR ==========
% Santa quadrotor body (in red)
santa_x = 8;
santa_y = 18;

% Quadrotor body (sleigh-shaped) - store handle
sleigh_x = [-1.5, -1.2, 1.2, 1.5, 1.2, -1.2, -1.5];
sleigh_y = [-0.5, 0.5, 0.5, -0.5, -0.8, -0.8, -0.5];
santa_body = fill(ax, santa_x + sleigh_x, santa_y + sleigh_y, [0.9 0.1 0.1], ...
    'EdgeColor', [0.7 0.1 0.1], 'LineWidth', 2);

% Quadrotor arms (in brown)
arm_length = 1.5;
arm_positions = [1 1; 1 -1; -1 1; -1 -1];
for i = 1:4
    x_arm = [santa_x, santa_x + arm_positions(i,1)*arm_length];
    y_arm = [santa_y, santa_y + arm_positions(i,2)*arm_length];
    plot(ax, x_arm, y_arm, '-', 'Color', [0.5 0.3 0.1], 'LineWidth', 3);
end

% Propellers (as presents!)
prop_colors = {'g', 'b', 'y', 'm'};
prop_handles = [];
for i = 1:4
    prop_x = santa_x + arm_positions(i,1)*arm_length;
    prop_y = santa_y + arm_positions(i,2)*arm_length;
    
    % Present box
    h = fill(ax, prop_x + [-0.3 0.3 0.3 -0.3], prop_y + [-0.3 -0.3 0.3 0.3], ...
        prop_colors{i}, 'EdgeColor', 'k', 'LineWidth', 1);
    prop_handles = [prop_handles, h];
    
    % Ribbon
    plot(ax, [prop_x-0.3, prop_x+0.3], [prop_y, prop_y], 'w-', 'LineWidth', 2);
    plot(ax, [prop_x, prop_x], [prop_y-0.3, prop_y+0.3], 'w-', 'LineWidth', 2);
end

% Santa hat on quadrotor
hat_x = [santa_x-1, santa_x, santa_x+1, santa_x, santa_x-1];
hat_y = [santa_y+0.5, santa_y+2, santa_y+0.5, santa_y+0.8, santa_y+0.5];
santa_hat = fill(ax, hat_x, hat_y, [0.9 0.1 0.1], 'EdgeColor', [0.7 0.1 0.1], 'LineWidth', 2);

% White fur on hat
fill(ax, [santa_x-0.8, santa_x+0.8, santa_x], ...
    [santa_y+0.5, santa_y+0.5, santa_y+0.8], 'w', 'EdgeColor', 'none');

% Pom-pom
santa_pompom = plot(ax, santa_x, santa_y+2, 'wo', 'MarkerSize', 10, 'MarkerFaceColor', 'w');

%% ========== CREATE REINDEER QUADROTOR ==========
reindeer_x = -8;
reindeer_y = 16;

% Reindeer body (brown quadrotor)
deer_body_x = [-1, -0.8, 0.8, 1, 0.8, -0.8, -1];
deer_body_y = [-0.4, 0.4, 0.4, -0.4, -0.6, -0.6, -0.4];
reindeer_body = fill(ax, reindeer_x + deer_body_x, reindeer_y + deer_body_y, ...
    [0.5 0.3 0.1], 'EdgeColor', [0.4 0.2 0.1], 'LineWidth', 2);

% Antlers (as special propellers)
antler_angles = [30, -30, 150, -150];
for i = 1:4
    antler_len = 1.2;
    if i > 2
        antler_len = 0.8; % Shorter back antlers
    end
    x_ant = [reindeer_x, reindeer_x + antler_len*cosd(antler_angles(i))];
    y_ant = [reindeer_y, reindeer_y + antler_len*sind(antler_angles(i))];
    plot(ax, x_ant, y_ant, '-', 'Color', [0.4 0.2 0], 'LineWidth', 2);
end

% Red nose
reindeer_nose = plot(ax, reindeer_x + 1.2, reindeer_y, 'ro', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'none');

%% ========== CREATE SNOW ==========
% Ground snow
snow_ground_x = linspace(-15, 15, 100);
snow_ground_y = -3 + 0.5 * sin(snow_ground_x * 0.5);
fill(ax, [-15, 15, 15, -15], [-5, -5, snow_ground_y(1), snow_ground_y(1)], ...
    [0.95 0.95 1], 'EdgeColor', 'none');

% Falling snow
n_snowflakes = 50;
snow_x = 30 * rand(1, n_snowflakes) - 15;
snow_y = 30 * rand(1, n_snowflakes);
snow_size = 5 + 15 * rand(1, n_snowflakes);

snowflakes = scatter(ax, snow_x, snow_y, snow_size, 'w', 'filled');
set(snowflakes, 'MarkerFaceAlpha', 0.8);

%% ========== ADD TEXT MESSAGES ==========
% Main message
text(ax, 0, 22, 'MERRY CHRISTMAS!', ...
    'HorizontalAlignment', 'center', 'FontSize', 36, 'FontWeight', 'bold', ...
    'Color', [1 0.2 0.2], 'FontName', 'Cambria');

% Subtitle
text(ax, 0, 20, 'From the Quadrotor Team', ...
    'HorizontalAlignment', 'center', 'FontSize', 18, 'FontWeight', 'bold', ...
    'Color', [0.2 0.8 1], 'FontName', 'Cambria');

% EKF joke
text(ax, 0, -2, 'Even Santa uses EKF for precise gift delivery!', ...
    'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold', ...
    'Color', [0.8 0.8 0.2], 'FontName', 'Comic Sans MS');

% Project Results Summary
results_text = {
    'Project Results Summary:';
    '• EKF successfully estimates 6 states from 3 measurements';
    '• Filter robust up to 3x normal noise levels';
    '• RMSE < 0.1 in optimal conditions';
    '• Excellent noise rejection performance';
};
text(ax, -14, 24, results_text, ...
    'HorizontalAlignment', 'left', 'FontSize', 10, 'FontWeight', 'bold', ...
    'Color', [0.8 0.8 0.5], 'FontName', 'Arial');

% Credits
text(ax, 13, -4, '© 2024 Quadrotor Simulation Project', ...
    'HorizontalAlignment', 'right', 'FontSize', 10, ...
    'Color', [0.6 0.6 0.6], 'FontName', 'Arial');

%% ========== ADD ANIMATION ==========
% Animation controls
uicontrol('Parent', fig, 'Style', 'pushbutton', ...
    'String', '🎄 Animate Christmas!', ...
    'Position', [20 20 150 40], ...
    'BackgroundColor', [0.9 0.2 0.2], 'ForegroundColor', 'white', ...
    'FontSize', 12, 'FontWeight', 'bold', ...
    'Callback', @animate_christmas);

    function animate_christmas(~, ~)
        % Simple animation without brighten function
        fprintf('🎅 Starting Christmas animation...\n');
        
        % Store original positions
        orig_snow_x = snow_x;
        orig_snow_y = snow_y;
        orig_star_size = star_size;
        
        % Animation loop
        for frame = 1:50
            % Update snow positions
            snow_y = snow_y - 0.3;
            snow_y(snow_y < -5) = 30; % Reset snowflakes that fall off
            
            % Make stars twinkle
            star_size = orig_star_size .* (0.8 + 0.4 * sin(frame/10 + (1:n_stars)));
            
            % Update scatter plots
            set(snowflakes, 'YData', snow_y);
            set(stars, 'SizeData', star_size);
            
            % Blink tree ornaments (simple version)
            if mod(frame, 5) == 0
                for i = 1:length(ornament_handles)
                    if rand() > 0.5
                        current_color = get(ornament_handles(i), 'MarkerFaceColor');
                        % Toggle between normal and bright
                        if mean(current_color) < 0.5
                            set(ornament_handles(i), 'MarkerSize', 15); % Make bigger
                        else
                            set(ornament_handles(i), 'MarkerSize', 12); % Back to normal
                        end
                    end
                end
            end
            
            % Move Santa quadrotor slightly (sin wave)
            santa_offset = 0.2 * sin(frame/10);
            
            % Move Santa body
            current_x = get(santa_body, 'XData');
            set(santa_body, 'XData', current_x + santa_offset);
            
            % Move Santa hat
            current_hat_x = get(santa_hat, 'XData');
            set(santa_hat, 'XData', current_hat_x + santa_offset);
            
            % Move Santa pom-pom
            current_pompom_x = get(santa_pompom, 'XData');
            set(santa_pompom, 'XData', current_pompom_x + santa_offset);
            
            % Blink reindeer nose
            if mod(frame, 3) == 0
                current_size = get(reindeer_nose, 'MarkerSize');
                set(reindeer_nose, 'MarkerSize', current_size + 2);
            elseif mod(frame, 3) == 1
                current_size = get(reindeer_nose, 'MarkerSize');
                set(reindeer_nose, 'MarkerSize', current_size - 2);
            end
            
            % Update display
            drawnow;
            pause(0.1);
        end
        
        % Reset to original state
        snow_y = orig_snow_y;
        set(snowflakes, 'YData', snow_y);
        set(stars, 'SizeData', orig_star_size);
        
        % Reset ornament sizes
        for i = 1:length(ornament_handles)
            set(ornament_handles(i), 'MarkerSize', 12);
        end
        
        % Reset Santa position
        set(santa_body, 'XData', santa_x + sleigh_x);
        set(santa_hat, 'XData', hat_x);
        set(santa_pompom, 'XData', santa_x);
        set(reindeer_nose, 'MarkerSize', 8);
        
        fprintf('🎄 Animation complete! Merry Christmas! 🎁\n');
        
        % Final message
        text(ax, 0, 18, 'HAPPY HOLIDAYS!', ...
            'HorizontalAlignment', 'center', 'FontSize', 24, 'FontWeight', 'bold', ...
            'Color', [0.2 0.9 0.2], 'FontName', 'Cambria', ...
            'BackgroundColor', 'white', 'EdgeColor', 'green');
    end

%% ========== ADD CHRISTMAS MUSIC NOTE ==========
% Musical notes
notes_x = [-12, -11, -10, -9, -8, -7];
notes_y = [1, 2, 1.5, 3, 2.5, 2];
scatter(ax, notes_x, notes_y, 100, 'y', 'filled', '^');
text(ax, -11.5, 0.5, 'Jingle Bells!', 'Color', 'y', 'FontSize', 12, 'FontWeight', 'bold');

% Add a final quadrotor gift under the tree
gift_x = [1, 2, 2, 1];
gift_y = [1.5, 1.5, 2.5, 2.5];
fill(ax, gift_x, gift_y, 'r', 'EdgeColor', 'k', 'LineWidth', 2);
fill(ax, gift_x+1, gift_y, 'g', 'EdgeColor', 'k', 'LineWidth', 2);
text(ax, 1.5, 2, 'EKF', 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

fprintf('\n═══════════════════════════════════════════════════\n');
fprintf('          MERRY CHRISTMAS & HAPPY HOLIDAYS!          \n');
fprintf('         From the Quadrotor Simulation Team          \n');
fprintf('═══════════════════════════════════════════════════\n\n');
end