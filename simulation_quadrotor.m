function [state, output, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, x0)

% Three functions combined into one

% getting all the quadrotor characteristics
m = rotor_data.m; 
r = rotor_data.r; 
I = rotor_data.I; 
g = rotor_data.g; 
dt = rotor_data.dt;
C = rotor_data.C;
A = rotor_data.A;

if nargin == 4
    x0 = zeros(1,6);
end

% Initialize output and states based on input parameters
output.clean = zeros(length(time), size(C, 1));
output.real = zeros(length(time), size(C, 1));
output.filtered = zeros(length(time), size(C, 1));
output.running = zeros(length(time), size(C, 1));%RUNNING

state.clean = zeros(length(time), size(A, 1));
state.real = zeros(length(time), size(A, 1));
state.estimate = zeros(length(time), size(A, 1));
state.running = zeros(length(time), size(A, 1));%RUNNING

% Simulate the system dynamics over the specified time

% Set initial state
state.clean(1, :) = x0; 
state.real(1, :) = x0; 
state.estimate(1, :) = x0;
state.running(1, :) = x0;%RUNNING

output.clean(1, :) = C*state.clean(1, :)';
output.real(1, :) = output.clean(1, :);
output.filtered(1, :) = output.clean(1, :);
output.running(1, :) = output.clean(1, :);%RUNNING

% Filter data

P = eye(6); % Initial Uncertainty
% Q: Process Noise Covariance (Trust in Physics)
% We use a small value to allow the model to drive the smoothness
Q = eye(6) * 1e-5; 
% R: Measurement Noise Covariance (Trust in Sensors)
% We set this higher than actual noise to filter out the jitters
R = eye(3) * 1e-1;

% Running 
window_size = 10; % You can adjust this window size

for t = 2:length(time)
    
%%% Perfect simulation
    
    % update theta
    theta_clean = state.clean(t - 1, 5);

    % Update states based on linear dynamics, this works for everything
    % except dx and dy
    delta_x_clean(1) = state.clean(t - 1, 2)*dt;
    delta_x_clean(3) = state.clean(t - 1, 4)*dt;
    delta_x_clean(5) = state.clean(t - 1, 6)*dt;
    delta_x_clean(6) = (r / I * control_input(t, 1) - r / I * control_input(t, 2)) * dt;

    % Now, non-linear part
    delta_x_clean(2) = (-sin(theta_clean)*(control_input(t, 1)+control_input(t, 2))/m) * dt;
    delta_x_clean(4) = (cos(theta_clean)*(control_input(t, 1)+control_input(t, 2))/m - g) * dt;
     

    % Lastly, updating states
    state.clean(t, :) = state.clean(t-1, :) + delta_x_clean(:)';
    output.clean(t, :) = (C * state.clean(t, :)')';

%%% Real-world simulation
    
    % update theta
    theta_real = state.real(t - 1, 5);

    % Update states based on linear dynamics, this works for everything
    % except dx and dy
    delta_x_real(1) = state.real(t - 1, 2)*dt;
    delta_x_real(3) = state.real(t - 1, 4)*dt;
    delta_x_real(5) = state.real(t - 1, 6)*dt;
    delta_x_real(6) = (r / I * control_input(t, 1) - r / I * control_input(t, 2)) * dt;

    % Now, non-linear part
    delta_x_real(2) = (-sin(theta_real)*(control_input(t, 1)+control_input(t, 2))/m) * dt;
    delta_x_real(4) = (cos(theta_real)*(control_input(t, 1)+control_input(t, 2))/m - g) * dt;

    % getting all the noise characteristics
    state_noise = noise_data.state_noise_amp * randn(1, size(A, 1));
    output_noise = noise_data.output_noise_amp * randn(1, size(C, 1));

    state.real(t, :) = state.real(t - 1, :) + delta_x_real(:)' + state_noise;
    output.real(t, :) = (C*state.real(t, :)')' + output_noise;

%%% Filter
% Filter uses perfect physics and sensor measurements, but we do not pass
% real state to it.

    % PREDICTION STEP

    % update theta
    theta_estimate = state.estimate(t - 1, 5);

    % Update states based on linear dynamics, this works for everything
    % except dx and dy
    delta_x_estimate(1) = state.estimate(t - 1, 2)*dt;
    delta_x_estimate(3) = state.estimate(t - 1, 4)*dt;
    delta_x_estimate(5) = state.estimate(t - 1, 6)*dt;
    delta_x_estimate(6) = (r / I * control_input(t, 1) - r / I * control_input(t, 2)) * dt;

    % Now, non-linear part
    delta_x_estimate(2) = (-sin(theta_estimate)*(control_input(t, 1)+control_input(t, 2))/m) * dt;
    delta_x_estimate(4) = (cos(theta_estimate)*(control_input(t, 1)+control_input(t, 2))/m - g) * dt;
    
    state.estimate(t, :) = state.estimate(t - 1, :) + delta_x_estimate(:)';

    % Predict covariance

    F = eye(6) + dt * [0 1 0 0 0 0;
                       0 0 0 0 -cos(theta_estimate)*(control_input(t, 1)+control_input(t, 2))/m 0;
                       0 0 0 1 0 0;
                       0 0 0 0 -sin(theta_estimate)*(control_input(t, 1)+control_input(t, 2))/m 0;
                       0 0 0 0 0 1;
                       0 0 0 0 0 0];
                       
    P_prediction = F * P * F' + Q;
    
    % CORRECTION STEP
    
    % 1. Calculate Measurement Residual 
    measurement_residual = output.real(t, :)' - C * state.estimate(t, :)';
    measurement_residual(2) = mod(measurement_residual(2) + pi, 2*pi) - pi;

    % 2. Calculate Kalman Gain
    S = C * P_prediction * C' + R;
    K = P_prediction * C' / S;
    
    % 3. Update State Estimate
    state.estimate(t, :) = state.estimate(t, :) + (K * measurement_residual)';
    output.filtered(t, :) = C * state.estimate(t, :)';

    % 4. Update Covariance
    P = (eye(6) - K * C) * P_prediction;


    % RUNNING 
    % Determine the start of the window
    window_start = max(1, t - window_size);
    
    % Average the noisy measurements ('real' outputs) over the window
    output.running(t, :) = mean(output.real(window_start:t, :), 1);
    
    % update theta
    theta_running = output.running(t, 2);
    
    state.running(t, 3) = output.running(t, 1); % Overwrite y (State 3)
    state.running(t, 5) = output.running(t, 2); % Overwrite theta (State 5)
    state.running(t, 6) = output.running(t, 3); % Overwrite theta_dot (State 6)

    % For the states, we apply the same averaging to the noisy running states
    % Update states based on linear dynamics, this works for everything
    % except dx and dy
    delta_x_running(1) = state.running(t-1, 2) * dt;

    % Now, non-linear part
    delta_x_running(2) = (-sin(theta_running)*(control_input(t, 1)+control_input(t, 2))/m) * dt;
    delta_x_running(4) = (cos(theta_running)*(control_input(t, 1)+control_input(t, 2))/m - g) * dt;

    state.running(t, [1 2 4]) = state.running(t - 1, [1 2 4]) + delta_x_running([1 2 4]);
    
end

%RUNNING
errors.output_clean_VS_running_total = output.running - output.clean;
errors.states_real_VS_running = state.real - state.running;

errors.output_clean_VS_real_total = output.real - output.clean;
errors.output_clean_VS_filtered_total = output.filtered - output.clean;
errors.output_real_VS_filtered_total = output.filtered - output.real;

errors.states_clean_VS_real_total = state.real - state.clean;
errors.states_real_VS_estimate = state.real - state.estimate;
errors.state_real_VS_output_real = output.real - state.real(:, [3, 5, 6]);
errors.state_real_VS_output_filtered = output.filtered - state.real(:, [3, 5, 6]);

% Calculate the Mean Squared Error for each column (state variable)
num_states = size(errors.states_real_VS_estimate, 2);
rmse_values = zeros(1, num_states);
rmse_running = zeros(1, num_states);

for i = 1:num_states
    % RMSE = sqrt(mean(error^2))
    rmse_values(i) = sqrt(mean(errors.states_real_VS_estimate(:, i).^2));
    %RUNNING
    rmse_running(i) = sqrt(mean(errors.states_real_VS_running(:, i).^2));
end

% Store the RMSE values in the errors structure
% state variables are typically: [x, dx, y, dy, theta, dtheta]
errors.rmse_states = rmse_values;
%RUNNING
errors.rmse_running = rmse_running;
end