function [Kalman_prediction] = kalman_filter(A,C,u,time,rotor_data,x0,Ys2)

% Quadrotor characteristics
m = rotor_data.m;
dt = rotor_data.dt;

% Inititalize output
Kalman_prediction = zeros(size(A,1),length(time)); % System state by Kalman filter

%Initialize filter
Kalman_prediction(:,1) = x0;
R = eye(3)*10^-3; % Measurement noise variance
Q = eye(6)*10^-3; % Process noise variance
P0 = eye(6); % Initial variance

for i = 2:length(time)

    % Prediction stage
    F = eye(6)+dt*[0  1  0  0   0   0;
                   0  0  0  0   -cos(Kalman_prediction(5,i-1))*(u(i-1,1)+u(i-1,2))/m   0;
                   0  0  0  1   0   0;
                   0  0  0  0   -sin(Kalman_prediction(5,i-1))*(u(i-1,1)+u(i-1,2))/m   0;
                   0  0  0  0   0   1;
                   0  0  0  0   0   0]; % F matrix depends on the state, so this is the way
    x_est = F*Kalman_prediction(:,i-1);
    P_est = F * P0 * F' + Q; % P_i^-
   
    % Correction stage
    
    k = P_est * C' / (C * P0 * C' + R); % Kalman gain (6 by 3 matrix)
    Kalman_prediction(:,i) = (eye(6)-k * C) * x_est + k * Ys2(i,:)';
    P0 = (eye(6) - k * C) * P_est; % Technically iteration stage, but it is united with the last correction step
end
end