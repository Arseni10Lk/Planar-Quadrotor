clc;
clear; 
close all;

rng(45);

%% DEFINE VARIABLES
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

%% STEP 6: DEFINE NOISE AMPLITUDE & robustness testing

x0 = [0;0;1;0;0;0];
[noise_data_all, error_all] = robustness(rotor_data, control_input, time, x0);



%% STEP 7: SIMULATION

state_labels = {'x', 'x-dot', 'y', 'y-dot', 'theta', 'theta-dot'};
fprintf('\n RMSE Values (State: [x, x-dot, y, y-dot, theta, theta-dot]) \n');

% 1: Optimal conditions
[state1, output1, errors1] = simulation_quadrotor(rotor_data, control_input, noise_data_all.noise_data1, time, x0);
fprintf('Case 1 : ');
disp(errors1.rmse_states);
plot_quadrotor_results(time, state1, output1, rotor_data.C, errors1);
sgtitle('Case 1: Optimal conditions (Low Noise)');

% 2: Regular conditions
[state2, output2, errors2] = simulation_quadrotor(rotor_data, control_input, noise_data_all.noise_data2, time, x0);
fprintf('Case 2 : ');
disp(errors2.rmse_states);
plot_quadrotor_results(time, state2, output2, rotor_data.C, errors2);
sgtitle('Case 2: Regular conditions (Regular Noise)');

% 3: Inconvenient conditions
[state3, output3, errors3] = simulation_quadrotor(rotor_data, control_input, noise_data_all.noise_data3, time, x0);
fprintf('Case 3 : ');
disp(errors3.rmse_states);
plot_quadrotor_results(time, state3, output3, rotor_data.C, errors3);
sgtitle('Case 3: Inconvenient conditions (Medium-High Noise)');

% 4: Very inconvenient conditions
[state4, output4, errors4] = simulation_quadrotor(rotor_data, control_input, noise_data_all.noise_data4, time, x0);
fprintf('Case 4 : ');
disp(errors4.rmse_states);
plot_quadrotor_results(time, state4, output4, rotor_data.C, errors4);
sgtitle('Case 4: Very inconvenient conditions (High Noise)');

% 5: No process noise / high measurement noise
[state5, output5, errors5] = simulation_quadrotor(rotor_data, control_input, noise_data_all.noise_data5, time, x0);
fprintf('Case 5 : ');
disp(errors5.rmse_states);
plot_quadrotor_results(time, state5, output5, rotor_data.C, errors5);
sgtitle('Case 5: NO process noise // high measurement noise');

% 6: High process noise / no measurement noise
[state6, output6, errors6] = simulation_quadrotor(rotor_data, control_input, noise_data_all.noise_data6, time, x0);
fprintf('Case 6 : ');
disp(errors6.rmse_states);
plot_quadrotor_results(time, state6, output6, rotor_data.C, errors6);
sgtitle('Case 6: high process noise // NO measurement noise');

% 7: Divergence case
[state7, output7, errors7] = simulation_quadrotor(rotor_data, control_input, noise_data_all.noise_data7, time, x0);
fprintf('Case 7 : ');
disp(errors7.rmse_states);
plot_quadrotor_results(time, state7, output7, rotor_data.C, errors7);
sgtitle('Case 7: Clear divergence');
