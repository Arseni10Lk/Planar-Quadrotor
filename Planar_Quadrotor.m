clc;
clear; 
close all;

rng(45);

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

%% STEP 4: DEFINE TIME VECTOR
time = 0:dt:t_max;  

%% STEP 5: DEFINE CONTROL INPUTS
% Create CU1 and CU2 (time-varying control inputs)
CU1 = 3 + 0.001*cos(2*time);  
CU2 = 3 + 0.001*sin(2*time); 

% Combine for lsim
control_input = [CU1; CU2]';  

%% STEP 6: DEFINE NOISE AMPLITUDE

noise_data1.state_noise_amp = 0.002; % Process noise amplitude
noise_data1.output_noise_amp = 0.025; % Measurement noise amplitude

noise_data2.state_noise_amp = 0.004; % Process noise amplitude
noise_data2.output_noise_amp = 0.05; % Measurement noise amplitude.

noise_data3.state_noise_amp = 0.014; % Process noise amplitude
noise_data3.output_noise_amp = 0.15; % Measurement noise amplitude.

noise_data4.state_noise_amp = 0.08; % Process noise amplitude
noise_data4.output_noise_amp = 0.25; % Measurement noise amplitude.

%% STEP 7: SIMULATION

x0 = [0;0;1;0;0;0];

state_labels = {'x', 'x-dot', 'y', 'y-dot', 'theta', 'theta-dot'};
fprintf('\n RMSE Values (State: [x, x-dot, y, y-dot, theta, theta-dot]) \n');

% 1: Lowest Noise
[state1, output1, errors1] = simulation_quadrotor(rotor_data, control_input, noise_data1, time, x0);

fprintf('Case 1 : ');
disp(errors1.rmse_states);
plot_quadrotor_results(time, state1, output1, rotor_data.C, errors1);
sgtitle('Case 1: Low Noise');

% 2: Medium-Low Noise
[state2, output2, errors2] = simulation_quadrotor(rotor_data, control_input, noise_data2, time, x0);
fprintf('Case 2 : ');
disp(errors2.rmse_states);
plot_quadrotor_results(time, state2, output2, rotor_data.C, errors2);
sgtitle('Case 2: Medium-Low Noise ');

% 3: Medium-High Noise 
[state3, output3, errors3] = simulation_quadrotor(rotor_data, control_input, noise_data3, time, x0);
fprintf('Case 3 : ');
disp(errors3.rmse_states);
plot_quadrotor_results(time, state3, output3, rotor_data.C, errors3);
sgtitle('Case 3: Medium-High Noise ');

% 4: Highest Noise
[state4, output4, errors4] = simulation_quadrotor(rotor_data, control_input, noise_data4, time, x0);
fprintf('Case 4 : ');
disp(errors4.rmse_states);
plot_quadrotor_results(time, state4, output4, rotor_data.C, errors4);
sgtitle('Case 4: High Noise ');


