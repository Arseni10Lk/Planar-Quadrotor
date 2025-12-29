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
% Create CU1 and CU2 (basic case)
CU1 = 3 * (1 + 0.001*cos(2*time));  
CU2 = 3 * (1 + 0.001*sin(2*time));

% Create CU3 and CU4 (horizontal flight recovery)
CU3 = zeros(size(time));
CU4 = zeros(size(time));

for t = 0:109 % roll up
    CU3(t + 1) = 4.05 * (1 + 0.001*cos(2 * 0.01 * t));
    CU4(t + 1) = 4 * (1 + 0.001*sin(2 * 0.01 * t));
end
for t = 110:219  % stop rotation
    CU3(t + 1) = 4 * (1 + 0.001*cos(2 * 0.01 * t));
    CU4(t + 1) = 4.05 * (1 + 0.001*sin(2 * 0.01 * t));
end
for t = 220:length(time) % fly up
    CU3(t + 1) = 4 * (1 + 0.001*cos(2 * 0.01 * t));
    CU4(t + 1) = 4 * (1 + 0.001*sin(2 * 0.01 * t));
end

% Create CU5 and CU6 (360 roll)
CU5 = zeros(size(time));
CU6 = zeros(size(time));

for t = 0:199 % iniiate roll
    CU5(t + 1) = 4.04 * (1 + 0.001*cos(2 * 0.01 * t));
    CU6(t + 1) = 4 * (1 + 0.001*sin(2 * 0.01 * t));
end 
for t = 200:299 % let it roll
    CU5(t + 1) = 0 * (1 + 0.001*cos(2 * 0.01 * t));
    CU6(t + 1) = 0 * (1 + 0.001*sin(2 * 0.01 * t));
end 
for t = 300:500 % stop rotation
    CU5(t + 1) = 0 * (1 + 0.001*cos(2 * 0.01 * t));
    CU6(t + 1) = 0.04 * (1 + 0.001*sin(2 * 0.01 * t));
end
for t = 501:length(time) % recover vertically
    CU5(t + 1) = 5 * (1 + 0.001*cos(2 * 0.01 * t));
    CU6(t + 1) = 5 * (1 + 0.001*sin(2 * 0.01 * t));
end 

% Create CU7 and CU8 (straight fall recovery)
CU7 = 3.2 * (1 + 0.001*cos(2*time));
CU8 = 3.2 * (1 + 0.001*sin(2*time));

% Combine for lsim
control_input.basic = [CU1; CU2]';
control_input.horizontal = [CU3; CU4]';
control_input.roll = [CU5; CU6]';
control_input.fall = [CU7; CU8]';


%% STEP 6: DEFINE NOISE AMPLITUDE & robustness testing

initial_state.basic = [0;0;1;0;0;0]; % basic case
initial_state.horizontal = [0;3;10;0;-pi/2;0]; % horizontal flight recovery
initial_state.roll = [0;0;50;5;0;0]; % roll 
initial_state.fall = [0;-1;15;-3;-(pi/2-atan(3/1));0]; % straight fall recovery

noise_data.state_noise_amp = 0.003;
noise_data.output_noise_amp = 0.02;

%% STEP 7: SIMULATION

[states, output, error] = simulation_quadrotor(rotor_data, control_input.roll, noise_data, time, initial_state.roll);
%% plot_quadrotor_results(time, states, output, C, error);

plot_quadrotor_enhanced(time, states, output, C, error);

%%merry_christmas_quadrotor();

[rmse_mat, noise_mat, div_data] = robustness(rotor_data, control_input.roll, time, initial_state.roll);

plot_robustness_results(rmse_mat, noise_mat, div_data);