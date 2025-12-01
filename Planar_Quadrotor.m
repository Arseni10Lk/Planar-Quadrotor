clc;
clear; 
close all;

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

D = zeros(3,2);  

%% STEP 3: CREATE STATE-SPACE OBJECT
quadrotor_sys = ss(A, B, C, D);


%% STEP 4: DEFINE TIME VECTOR
time = 0:dt:t_max;  

%% STEP 5: DEFINE CONTROL INPUTS
% Create CU1 and CU2 (time-varying control inputs)
CU1 = 2.5 + 0.001*cos(2*time);  
CU2 = 2.5 + 0.001*sin(2*time); 

% Combine for lsim
control_input = [CU1; CU2]';  

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
[output1, ~] = quadrotor_clean_physics_sim(A, C, control_input, time, rotor_data, []);
% system starts from zero

% Plot the outputs (measured states)
plot(time, output1(:,1), 'b-', 'LineWidth', 2); hold on;
plot(time, rad2deg(output1(:,2)), 'r-', 'LineWidth', 2);
plot(time, output1(:,3), 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Output');
title('System Response to Control Inputs (Zero Initial Conditions)');
legend('y-position', 'theta', 'theta-dot', 'Location', 'best');
grid on;

%% FIGURE 3: PLOT OUTPUT DATA

% Add noise here. For now, it is identical to Figure 2

figure(3);
% This is the same as Figure 2 but showing the data extraction
[Ys, states1] = quadrotor_clean_physics_sim(A, C, control_input, time, rotor_data, []);

plot(time, Ys(:,1), 'b-', 'LineWidth', 2); hold on;
plot(time, rad2deg(Ys(:,2)), 'r-', 'LineWidth', 2);
plot(time, Ys(:,3), 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Output');
title('Output Data: Ys vs Ti (Zero Initial Conditions)');
legend('y-measured', 'theta-measured', 'theta-dot-measured', 'Location', 'best');
grid on;

%% FIGURE 4: SYSTEM RESPONSE WITH INITIAL CONDITIONS
figure(4);

x0 = [0; 0; 1; 0; 0; 0];  % Start at x=0, y=1m, level hovering

%[Ys2, ~, states2] = lsim(quadrotor_sys, control_input, time, x0); - for
%linear systems
[Ys2, states2] = quadrotor_clean_physics_sim(A, C, control_input, time, rotor_data, x0);

% Plot all states to see complete evolution
subplot(2,1,1);
plot(time, states2(:,1), 'b-', 'LineWidth', 2); hold on;
plot(time, states2(:,3), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Position (m)');
title('Quadrotor Position (With Initial Conditions)');
legend('x-position', 'y-position', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(time, states2(:,5), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (rad)');
title('Pitch Angle Evolution'); grid on;

disp('=== Overal MATRICES ===');
disp('States matrix with initial conditions (first 5 rows):');
state_names = {'x_pos', 'x_vel', 'y_pos', 'y_vel', 'theta', 'theta_dot'};
results_table = array2table([time', states2], 'VariableNames', ['Time', state_names]);
disp(results_table(1:5,:));

disp('Output matrix with initial conditions (first 5 rows):');
output_names = {'y_measured', 'theta_measured', 'theta_dot_measured'};
output_table = array2table([time', Ys2], 'VariableNames', ['Time', output_names]);
disp(output_table(1:5,:));

%% STEP 6: KALMAN FILTER (at least it is expected)

Kalman_prediction = zeros(6,length(time)); % System state by Kalman filter
Kalman_prediction(:,1) = x0;
R = eye(3)*10^-3; % Measurement noise variance
Q = eye(6)*10^-3; % Process noise variance
P0 = eye(6); % Initial variance

for i = 2:length(time)

   
    % Prediction stage

    x_est = Kalman_prediction(:,i-1); % x_{i-1}
    F = eye(6)+dt*[0  1  0  0   0   0;
                   0  0  0  0   -cos(Kalman_prediction(5,i-1))*(control_input(i-1,1)+control_input(i-1,2))/m   0;
                   0  0  0  1   0   0;
                   0  0  0  0   -sin(Kalman_prediction(5,i-1))*(control_input(i-1,1)+control_input(i-1,2))/m   0;
                   0  0  0  0   0   1;
                   0  0  0  0   0   0]; % F matrix depends on the state, so this is the way
    P_est = F * P0 * transpose(F) + Q; % P_i^-
   
    % Correction stage
    
    k = P_est * transpose(C) / (C * P0 * transpose(C)+R); % Kalman gain (6 by 3 matrix)
    Kalman_prediction(:,i) = (eye(6)-k * C) * x_est + k * transpose(Ys2(i,:));
    P0 = (eye(6) - k * C) * P_est; % Technically iteration stage, but it is united with the last correction step
end
%% FIGURE 5: Temporary display of Kalman filter results
figure(5)

plot(time,Kalman_prediction(1,:)); hold on;
plot(time,Kalman_prediction(3,:));
plot(time,Kalman_prediction(5,:));
legend("x-position","y-position","theta angle");
