clc;
clear; 
close all;

rng(42);

%%DEFINE VARIABLES
m = 0.5;         % mass [kg] 
r = 0.15;        % distance from center to rotors [m]
I = 0.005;       % moment of inertia [kg*m^2]
g = 9.81;        % gravity [m/s^2]
dt = 0.01;       % time step [s] 
theta = 0;       % angle [rads]
u1 = 2.45;       % force [N]
u2 = 2.45;       % force [N]

% Structure definition
rotor_data.m = m;
rotor_data.r = r;
rotor_data.I = I;
rotor_data.g = g;
rotor_data.dt = dt;
rotor_data.theta = theta;
rotor_data.u1 = u1;
rotor_data.u2 = u2;

t_max = 10;      % simulation duration [s]

% A matrix - System dynamics
A = [0  1  0  0   0   0;
     0  0  0  0   -cos(theta)*(u1+u2)/m   0;
     0  0  0  1   0   0;
     0  0  0  0   -sin(theta)*(u1+u2)/m   0;
     0  0  0  0   0   1;
     0  0  0  0   0   0];

% It desscribes the states, so let's say it is rotor data 
rotor_data.A = A;

% B matrix - Control input
B = [0       0;
     0       0;
     0       0;
     1/m     1/m;
     0       0;
     r/I     -r/I]; 

% C matrix - Measurements
C = [0  0  1  0  0  0;
     0  0  0  0  1  0;
     0  0  0  0  0  1];

rotor_data.C = C; % It says what we measure, so let's say it is rotor data 
% (like what the sensors are)

D = zeros(3,2);  

%% STEP 3: CREATE STATE-SPACE OBJECT
quadrotor_sys = ss(A, B, C, D);


%% STEP 4: DEFINE TIME VECTOR
time = 0:dt:t_max;  

%% STEP 5: DEFINE CONTROL INPUTS
% Create CU1 and CU2 (time-varying control inputs)
CU1 = 3 + 0.001*cos(2*time);  
CU2 = 3 + 0.001*sin(2*time); 

% Combine for lsim
control_input = [CU1; CU2]';  

%% STEP 6: DEFINE NOISE AMPLITUDE

noise_data.state_noise_amp = 0.004; % Process noise amplitude
noise_data.output_noise_amp = 0.05; % Measurement noise amplitude

%% STEP 7: SIMULATION

x0 = [0;0;1;0;0;0];

[state, output, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, x0);

%% FIGURE 1: PLOT CONTROL INPUTS
figure(1);
plot(time, CU1, 'b-', 'LineWidth', 2); hold on;
plot(time, CU2, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Control Inputs: CU1 and CU2');
legend('CU1', 'CU2', 'Location', 'best');
grid on;

%% FIGURE 2: SYSTEM RESPONSE TO INPUTS
figure(2);

% Plot the outputs (measured states)
plot(time, output.clean(:,1), 'b-', 'LineWidth', 2); hold on;
plot(time, rad2deg(output.clean(:,2)), 'r-', 'LineWidth', 2);
plot(time, rad2deg(output.clean(:,3)), 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Output');
title('System Response to Control Inputs');
legend('y-position', 'theta', 'theta-dot', 'Location', 'best');
grid on;

%% FIGURE 3: PLOT OUTPUT DATA

figure(3);

plot(time, output.real(:,1), 'b-', 'LineWidth', 2); hold on;
plot(time, rad2deg(output.real(:,2)), 'r-', 'LineWidth', 2);
plot(time, rad2deg(output.real(:,3)), 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Output');
title('Output Data: Ys vs Ti');
legend('y-measured', 'theta-measured', 'theta-dot-measured', 'Location', 'best');
grid on;

%% FIGURE 4: SYSTEM RESPONSE
figure(4);

% Plot all states to see complete evolution
subplot(2,1,1);
plot(time, state.real(:,1), 'b-', 'LineWidth', 2); hold on;
plot(time, state.real(:,3), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Position (m)');
title('Quadrotor Position (clean) (With Initial Conditions)');
legend('x-position', 'y-position', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(time, rad2deg(state.real(:,5)), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Pitch Angle Evolution (clean)'); grid on;

disp('=== Overal MATRICES ===');
disp('States matrix with initial conditions (first 5 rows):');
state_names = {'x_pos', 'x_vel', 'y_pos', 'y_vel', 'theta', 'theta_dot'};
results_table = array2table([time', state.real], 'VariableNames', ['Time', state_names]);
disp(results_table(1:5,:));

disp('Output matrix with initial conditions (first 5 rows):');
output_names = {'y_measured', 'theta_measured', 'theta_dot_measured'};
output_table = array2table([time', output.real], 'VariableNames', ['Time', output_names]);
disp(output_table(1:5,:));

%% STEP 7: KALMAN FILTER

%% FIGURE 5: Noisy Simulation
figure(5)

subplot(2,1,1);
plot(time, state.real(:,1), 'b-', 'LineWidth', 2); hold on;
plot(time, state.real(:,3), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Position (m)');
title('Quadrotor Position (noisy) (With Initial Conditions)');
legend('x-position', 'y-position', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(time, rad2deg(state.real(:,5)), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Pitch Angle Evolution (noisy)'); grid on;
%% FIGURE 6: Temporary display of Kalman filter results
figure(6)

subplot(2,1,1);
plot(time, state.estimate(:,1), 'b-', 'LineWidth', 2); hold on;
plot(time, state.estimate(:,3), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Position (m)');
title('Quadrotor Position (With Initial Conditions) From Extended Kalman Filter');
legend('x-position', 'y-position', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(time, rad2deg(state.estimate(:,5)), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Pitch Angle Evolution'); grid on;

