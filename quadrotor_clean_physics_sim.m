function [output, states] = quadrotor_clean_physics_sim(A, C, u, time, rotor_data, x0)

% getting all the quadrotor characteristics
m = rotor_data.m; 
r = rotor_data.r; 
I = rotor_data.I; 
g = rotor_data.g; 
dt = rotor_data.dt;

if isempty(x0)
    x0 = zeros(1, 6);
end

% Initialize output and states based on input parameters
output = zeros(length(time), size(C, 1));
states = zeros(length(time), size(A, 1));

% Simulate the system dynamics over the specified time

states(1, :) = x0; % Set initial state
output(1, :) = C*states(1, :)';

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


    states(t, :) = states(t-1, :) + delta_x(:)';

    output(t, :) = C * states(t, :)'; % Compute output from states
end
end