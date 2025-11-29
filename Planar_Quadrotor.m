
%%DEFINE VARIABLES
m = 0.5;         % mass [kg] 
r = 0.15;        % distance from center to rotors [m]
I = 0.005;       % moment of inertia [kg*m^2]
g = 9.81;        % gravity [m/s^2]
dt = 0.01;       % time step [s] 
theta = 0;       % angle [rads]
u1 = 2.45;       % force [N]
u2 = 2.45;       % force [N]

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
     r/I     -r/I];  % Missing -r/I in original

% C matrix - Measurements
C = [0  0  1  0  0  0;
     0  0  0  0  1  0;
     0  0  0  0  0  1];

D = zeros(3,2);  % Noticed this was forgotten from before

%% STEP 3: CREATE STATE-SPACE OBJECT (Like video's sys = ss(A,B,C,D))
quadrotor_sys = ss(A, B, C, D);


%% STEP 4: DEFINE TIME VECTOR (Like video's time = 0:0.01:15)
time = 0:dt:10;  

%% STEP 5: DEFINE CONTROL INPUTS (Like video's control_U = 30*sin(2*time) - 10*cos(4*time))
% Create CU1 and CU2 (time-varying control inputs)
CU1 = 2.5 + 0.5*sin(2*time);  
CU2 = 2.5 + 0.3*cos(1.5*time); 

% Combine for lsim (like video's control_U but for 2 inputs)
control_input = [CU1; CU2]';  

%% FIGURE 1: PLOT CONTROL INPUTS (Like video's plot(time, control_U))
figure(1);
plot(time, CU1, 'b-', 'LineWidth', 2); hold on;
plot(time, CU2, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Control Inputs: CU1 and CU2');
legend('CU1', 'CU2', 'Location', 'best');
grid on;

%% FIGURE 2: SYSTEM RESPONSE TO INPUTS (Like video's lsim(sys, control_U, time))
figure(2);
[output1, time_data1, states1] = lsim(quadrotor_sys, control_input, time);
% system starts from zero

% Plot the outputs (measured states)
plot(time_data1, output1(:,1), 'b-', 'LineWidth', 2); hold on;
plot(time_data1, output1(:,2), 'r-', 'LineWidth', 2);
plot(time_data1, output1(:,3), 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Output');
title('System Response to Control Inputs (Zero Initial Conditions)');
legend('y-position', 'theta', 'theta-dot', 'Location', 'best');
grid on;

%% FIGURE 3: PLOT OUTPUT DATA (Like video's plot(Ti, Ys))
figure(3);
% This is the same as Figure 2 but showing the data extraction
[Ys, Ti] = lsim(quadrotor_sys, control_input, time);

plot(Ti, Ys(:,1), 'b-', 'LineWidth', 2); hold on;
plot(Ti, Ys(:,2), 'r-', 'LineWidth', 2);
plot(Ti, Ys(:,3), 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Output');
title('Output Data: Ys vs Ti (Zero Initial Conditions)');
legend('y-measured', 'theta-measured', 'theta-dot-measured', 'Location', 'best');
grid on;

%% FIGURE 4: SYSTEM RESPONSE WITH INITIAL CONDITIONS (Like video's lsim(sys, control_U, time, x0))
figure(4);

x0 = [0; 0; 1; 0; 0; 0];  % Start at x=0, y=1m, level hovering

[Ys2, Ti2, states2] = lsim(quadrotor_sys, control_input, time, x0);

% Plot all states to see complete evolution
subplot(2,1,1);
plot(Ti2, states2(:,1), 'b-', 'LineWidth', 2); hold on;
plot(Ti2, states2(:,3), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Position (m)');
title('Quadrotor Position (With Initial Conditions)');
legend('x-position', 'y-position', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(Ti2, states2(:,5), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (rad)');
title('Pitch Angle Evolution'); grid on;


disp('=== Overal MATRICES ===');
disp('States matrix with initial conditions (first 5 rows):');
state_names = {'x_pos', 'x_vel', 'y_pos', 'y_vel', 'theta', 'theta_dot'};
results_table = array2table([Ti2, states2], 'VariableNames', ['Time', state_names]);
disp(results_table(1:5,:));

disp('Output matrix with initial conditions (first 5 rows):');
output_names = {'y_measured', 'theta_measured', 'theta_dot_measured'};
output_table = array2table([Ti2, Ys2], 'VariableNames', ['Time', output_names]);
disp(output_table(1:5,:));