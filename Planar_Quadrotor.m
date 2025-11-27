
%%DEFINE VARIABLES
u1 = 1;        % force [N]
u2 = 1;        % force [N]
m = 1;         % mass [kg]
r = 1;         % distance [m]
I = 1;         % moment of inertia [kg*m^2]
g = 9.81;      % gravity [m/s^2]
dt = 0.1;        % increment of time [s]
theta = 1;     % angle [rads]

%%DEFINE MATRIX

A = [0  1  0  0             0           0,
     0  0  0  0  -cos(theta)*(u1+u2)/m  0,
     0  0  0  1             0           0,
     0  0  0  0  -sin(theta)*(u1+u2)/m  0,
     0  0  0  0             0           1,
     0  0  0  0             0           0]


F = [1   dt  0   0            0               0,
     0   1   0   0  -cos(theta)*(u1+u2)*dt/m  0,
     0   0   1   dt           0               0,
     0   0   0   1  -sin(theta)*(u1+u2)*dt/m  0,
     0   0   0   0            1              dt,
     0   0   0   0            0               1]


B = [0       0,
     0       0,
     0       0,
     1/m   1/m,
     0       0,
     r/I   r/I] 


C = [0   0   1   0   0   0,
     0   0   0   0   1   0,
     0   0   0   0   0   1]


H = [0       0       1       0       0       0,
     0       0       0       0       1       0,
     0       0       0       0       0       1]


