function [output, states, kalman] = quadrotor_NOISY_physics_sim(A, C, u, time, rotor_data, x0, state_noise_amp, measurement_noise_amp)

% getting all the quadrotor characteristics
m = rotor_data.m; 
r = rotor_data.r; 
I = rotor_data.I; 
g = rotor_data.g; 
dt = rotor_data.dt;

if isempty(x0)
    x0 = zeros(1, 6);
end

% Initialize output and states based on input parameters, kalman state vect
output = zeros(length(time), size(C, 1));
states = zeros(length(time), size(A, 1));
kalman = zeros(length(time), size(A, 1));


rng(2);
% Kalman filter definitions
P0 = eye(6); % Initial variance

% Simulate the system dynamics over the specified time

states(1, :) = x0; % Set initial state
output(1, :) = C*states(1, :)';
kalman(1, :) = x0;  

for t = 2:length(time)
    
    % update theta
    theta = states(t - 1, 5);

    % Update states based on linear dynamics, this works for everything
    % except dx and dy
    delta_x(1) = states(t - 1, 2)*dt;
    delta_x(3) = states(t - 1, 4)*dt;
    delta_x(5) = states(t - 1, 6)*dt;
    delta_x(6) = (r / I * u(t, 1) - r / I * u(t, 2)) * dt;

    % Now, non-linear part
    delta_x(2) = (-sin(theta)*(u(t, 1)+u(t, 2))/m) * dt;
    delta_x(4) = (cos(theta)*(u(t, 1)+u(t, 2))/m - g) * dt;

    state_noise = state_noise_amp * randn(1, size(A, 1));
    states(t, :) = states(t-1, :) + delta_x(:)' + state_noise;

    output_noise = measurement_noise_amp * randn(size(C, 1),1);
    output(t, :) = C * states(t, :)' + output_noise; % Compute output from states with added noise

    % Kalman update step

    [P0,kalman(t,:)] = kalman_filter(C, u(t,:), rotor_data, kalman(t-1,:), output(t,:), P0);
end
end
