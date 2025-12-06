function [Pi,Kalman_prediction] = kalman_filter(C,u,rotor_data,state,measurement,P0)

% Quadrotor characteristics
m = rotor_data.m;
dt = rotor_data.dt;

%Initialize filter
R = eye(3)*10^-1; % Measurement noise variance
Q = eye(6)*10^-3; % Process noise variance

% Prediction stage
F = eye(6)+dt*[0  1  0  0   0   0;
               0  0  0  0   -cos(state(5))*(u(1)+u(2))/m   0;
               0  0  0  1   0   0;
               0  0  0  0   -sin(state(5))*(u(1)+u(2))/m   0;
               0  0  0  0   0   1;
               0  0  0  0   0   0]; % F matrix depends on the state, so this is the way
x_est = F*state';
P_est = F * P0 * F' + Q; % P_i^-
   
% Correction stage
    
k = P_est * C' / (C * P_est * C' + R); % Kalman gain (6 by 3 matrix)
%Kalman_prediction = (x_est + k * (measurement' - (C * x_est)))';
Kalman_prediction = (eye(6)-k * C) * x_est + k * measurement';
Pi = (eye(6) - k * C) * P_est; % Technically iteration stage, but it is united with the last correction step
end
