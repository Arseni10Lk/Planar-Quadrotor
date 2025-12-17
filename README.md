# Planar Quadrotor 

## Project Overview
We are using an Extended Kalman Filter (EKF) to estimate the state of a 2D quadrotor using noisy sensor measurements. The EKF combines physics-based predictions with real sensor data to provide accurate state estimates.

![Planar Quadrotor Diagram](Planar_Quadrotor.png)

### The Challenge
- Sensors are noisy: IMU and altitude sensors provide unreliable, jittery measurements
- Physics is complex: Real quadrotor dynamics are nonlinear and complicated  
- Solution: Combine sensor data with mathematical models to get better estimates than either could provide alone

### Nonlinear System Equations
The quadrotor's movement is described by these nonlinear equations:

$$
\begin{aligned}
m\ddot{x} &= -(u_1 + u_2)\sin\theta \\
m\ddot{y} &= (u_1 + u_2)\cos\theta - mg \\
I\ddot{\theta} &= r(u_1 - u_2)
\end{aligned}
$$

Where:
- m: mass of the quadrotor
- I: moment of inertia  
- r: distance from center to rotors
- g: gravitational acceleration
- u₁, u₂: rotor thrust forces

## Problem Definition

The problem is defined in MATLAB using reasonable values for each variable:
```matlab
m = 0.5;         % mass [kg] 
r = 0.15;        % distance from center to rotors [m]
I = 0.005;       % moment of inertia [kg*m^2]
g = 9.81;        % gravity [m/s^2]
dt = 0.01;       % time step [s] 
theta = 0;       % angle [rads]
u1_max = 2.45;       % force max magnitude [N]
u2_max = 2.45;       % force max magnitude [N]
```

We use time-varying control inputs:

```matlab
u1 = u1_max + 0.001*cos(2*time);  
u2 = u2_max + 0.001*sin(2*time); 
```
These inputs create small oscillations around a thrust level slightly above hover, ensuring the quadrotor moves and tilts in a nontrivial way.

## System Definitions

**State Vector (6 elements):**

$$
\mathbf{x} = 
\begin{bmatrix}
x \\
\dot{x} \\
y \\
\dot{y} \\
\theta \\
\dot{\theta}
\end{bmatrix}
$$

**Input Vector (2 elements):** 

$$
\mathbf{u} = 
\begin{bmatrix}
u_1 \\
u_2
\end{bmatrix}
$$

## Non-linear perfect conditions simulation

Based on the three initial equations, the true dynamics are governed by:

$$
\begin{aligned}
\ddot{x} &= \frac{-(u_1 + u_2) \sin\theta}{m} \\
\ddot{y} &= \frac{(u_1 + u_2)\cos\theta}{m} - g \\
\ddot{\theta} &= \frac{r(u_1 - u_2)} {I}
\end{aligned}
$$

Since these equations do not account for any process noise, they correspond to "perfect conditions." The state vector for this clean trajectory is provided as `state.clean` in code.

This ensures the prediction respects the actual dynamics.

```matlab
  delta_x_clean(1) = state.clean(t - 1, 2)*dt;
  delta_x_clean(3) = state.clean(t - 1, 4)*dt;
  delta_x_clean(5) = state.clean(t - 1, 6)*dt;
  delta_x_clean(6) = (r / I * control_input(t, 1) - r / I * control_input(t, 2)) * dt;

  delta_x_clean(2) = (-sin(theta_clean)*(control_input(t, 1)+control_input(t, 2))/m) * dt;
  delta_x_clean(4) = (cos(theta_clean)*(control_input(t, 1)+control_input(t, 2))/m - g) * dt;
```

> **Note on gravity**: The term $$-mg$$ in the vertical force balance accounts for the gravitational pull.
> This results in a $$-g$$ term in the $$\ddot{y}$$ computation as seen in the code.

After those differences are obtained, the states can be updated

```matlab
state.clean(t, :) = state.clean(t-1, :) + delta_x_clean(:)';
output.clean(t, :) = (C * state.clean(t, :)')';
```

## Real-world trajectory

Real-world trajectory (`state.real`) has only one difference from the perfect conditions trajectory: there is process and measurement noise.

We use Gaussian noise, the amplitude of which was selected based on ... 

```matlab
state_noise = 0.004 * randn(1,6);
output_noise = 0.05 * randn(1,3);
```

All the equations governing the process stay the same, and the noise is added at the state update step:

```matlab
state.real(t, :) = state.real(t - 1, :) + delta_x_real(:)' + state_noise;
output.real(t, :) = (C*state.real(t, :)')' + output_noise;
```

## System Linearization for the Extended Kalman Filter

The Extended Kalman Filter requires a local linear approximation of the nonlinear dynamics to propagate uncertainty (covariance). This is achieved by computing the Jacobian matrices of the system equations with respect to the state and measurement variables.

**Jacobian for Covariance Propagation**
To update the error covariance, we compute the state transition Jacobian F, derived from the partial derivatives of the dynamics:
- $\frac{\partial \ddot{x}}{\partial \theta} = -\frac{\cos\theta}{m}(u_1 + u_2)$
  
- $\frac{\partial \ddot{y}}{\partial \theta} = -\frac{\sin\theta}{m}(u_1 + u_2)$
  
- $\frac{\partial \ddot{x}}{\partial u_1} = -\frac{\sin\theta}{m}$, $\frac{\partial \ddot{x}}{\partial u_2} = -\frac{\sin\theta}{m}$
  
  
- $\frac{\partial \ddot{y}}{\partial u_1} = \frac{\cos\theta}{m}$, $\frac{\partial \ddot{y}}{\partial u_2} = \frac{\cos\theta}{m}$

- $\frac{\partial \ddot{\theta}}{\partial u_1} = \frac{r}{I}$, $\frac{\partial \ddot{\theta}}{\partial u_2} = -\frac{r}{I}$

The Extended Kalman Filter (EKF) is used because our quadrotor system is nonlinear—its motion depends on sine and cosine of the angle $$\theta$$, so we can’t use a standard (linear) Kalman filter. Instead, the EKF **linearizes the system at each time step** using the current best estimate, then applies the regular Kalman update equations.

This gives the Jacobian A:

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & \frac{-cos(θ)(u₁+u₂)}{m} & 0 \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & \frac{-sin(θ)(u₁+u₂)}{m} & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

The discrete-time state transition matrix used in the EKF is:
$$F = I + A \Delta t$$, where I symbolizes the old state and A symbolizes the change in it.

$$
F = \begin{bmatrix} 
1 & \Delta t & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & \frac{-cos(θ)(u₁+u₂)}{m} \Delta t & 0 \\
0 & 0 & 1 & \Delta t & 0 & 0 \\
0 & 0 & 0 & 1 & \frac{-sin(θ)(u₁+u₂)}{m} \Delta t & 0 \\
0 & 0 & 0 & 0 & 1 & \Delta t \\
0 & 0 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
H = C = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

Our implementation follows the standard **predict–correct cycle**, and everything runs **in real time inside the simulation loop.**

### Prediction Step

First, we predict the next state using the true nonlinear physics — not a linear approximation. This state is saved as `state.estimate`.

This ensures the prediction respects the actual dynamics.

After the state is obtained for our current timestep. $F$ matrix is updated because it depends on the current angle (`theta_estimate`) and thrust (`control_input`). That's what makes it an *Extended* Kalman Filter.

We then predict error covariance:

$$P_{\text{prediction}} = F P F^T + Q$$

Where `Q = eye(6)*1e-5` represents the trust in the physics model.

### Correction step
Next, we correct the estimate using the latest noisy sensor data (`output.real`)
We compute the measurement residual—the difference between what the sensor says and what we predicted:
```matlab
measurement_residual = output.real(t,:)' - C * state.estimate(t,:)';
```
Then, we calculate the Kalman gain, which decides how much to trust the sensor vs. the prediction:

$$S = C P_{\text{prediction}} C^T + R$$
$$K = P_{\text{prediction}} C^T S^{-1}$$

Here,`R = eye(3)*1e-1` is larger than `Q`, meaning we trust the physics more than the sensors-which makes sense, since sensors are noisy.

Finally, we update the state and covariance:
```matlab
state.estimate(t,:) = state.estimate(t,:) + (K * measurement_residual)';
P = (eye(6) - K * C) * P_prediction;
```
This gives us a smoother, more accurate estimate than the raw sensor data.
  
## MATLAB Implementation Breakdown

The code we wrote is divided into three key parts:

### 1. Main Script (`Planar_Quadrotor.m`)

It defines the system and calls the simulation function:
```matlab
[state, output, errors] = simulation_quadrotor(rotor_data, control_input, noise_data, time, x0);
```
and then generates the final plots:
```matlab
plot_quadrotor_results(time, state, output, rotor_data.C, errors);
```

### 2. Core Simulation (`simulation_quadrotor.m`)

This function runs three parallel simulations in one loop, time step by time step:
- Clean trajectory (`state.clean`)
  Uses the true nonlinear dynamics with no noise. This is the Perfect Conditions.
- Real-world trajectory (`state.real`)
  Adds process noise to the state update and measurement noise to the outputs.
- EKF estimate (`state.estimate`)
  Runs the full EKF in real time:
  - Predicts the next state using the same nonlinear physics as the clean simulation.
  - Builds the Jacobian `F` matrix using the current estimated angle and thrust.
  - Uses noisy measurements (`output.real`) to correct the estimate.
  - Updates covariance with `Q = 1e-5*I` and `R = 1e-1*I`.

### 3. Plotting Function (`plot_quadrotor_results.m`)
Generates two figures showing Perfect Conditions, noisy measurements, and EFK estimates together for all measured states.

## Results & Visualization
### Figure 1: Measured States — Perfect Conditions vs Noisy vs EKF

![Measured States](Measured_state.png)

> The EKF successfully smooths out the sensor noise while closely tracking the true trajectory. Even though the red dots jump around due to 0.05 measurement noise, the green line remains stable.


### Figure 2: Full State Estimation — Perfect Conditions vs EKF
![Full State](Full_state.png)

> Even **unmeasured states** like horizontal position ($x$) and velocities ($\dot{x}, \dot{y}$) are reconstructed accurately. This is possible due to **dynamic coupling**: thrust affects both $y$ and $\theta$, which indirectly informs $x$-motion through $\sin\theta$ and $\cos\theta$ terms.

