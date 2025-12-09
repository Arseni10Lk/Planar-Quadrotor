# Planar Quadrotor 

## Project Overview
We are using an Extended Kalman Filter (EKF) to estimate the state of a 2D quadrotor using noisy sensor measurements. The EKF combines physics-based predictions with real sensor data to provide accurate state estimates.

![Planar Quadrotor Diagram](Planar_Quadrotor.png)

## Linear State-Space Equations Formulation 

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

### Linearization Process
We linearize the system by computing Jacobian matrices - taking partial derivatives of each equation with respect to all state variables and control inputs. This creates a linear approximation of the system dynamics that can be used for state estimation.

$$
x_k = F x_{k-1} + B_k u_k + w_k
$$

- $$B_k = B_s \Delta t$$
- $$W_k$$ ,Is process noise. This comes from imperfect physics modeling, eg, Unmodeled aerodynamics, Parameter uncertainties.

### State-Space Representation
The IMU measures total acceleration including gravity, so we explicitly subtract $g$ when processing the $\ddot{y}$ measurement. The standard linearized system of state-space equations:

$$
\begin{aligned}
\dot{\mathbf{x}} &= A\mathbf{x} + B_s\mathbf{u} + G \\
\mathbf{y} &= C\mathbf{x} + D\mathbf{u}
\end{aligned}
$$



### System Definitions

### State Vector (6 elements)
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

### Input Vector (2 elements) 
$$
\mathbf{u} = 
\begin{bmatrix}
u_1 \\
u_2
\end{bmatrix}
$$

### Observation Vector (3 elements)
Normally, an IMU provides linear accelerations and angular velocities in the body frame, which would require coordinate transformations. However, to simplify our observation model, we assume we can directly measure:

- Vertical position $$y$$ from an altitude sensor
- Pitch angle $$\theta$$ from the IMU  
- Angular velocity $$\dot{\theta}$$ from the IMU gyroscope

This allows us to use a simpler observation equation $$y_k = H x_k + v_k$$ where the measurements directly correspond to state variables.

$$
\mathbf{y} = 
\begin{bmatrix}
y \\
\theta \\
\dot{\theta} \\
\end{bmatrix}
$$

$$
y_k = H x_k + v_k
$$

 - $$v_k$$ is measurement noise (sensor errors, inaccuracies)



#### Linearized Continuous-Time Matrices
By taking partial derivatives, we obtain the **Jacobian matrices**:

* **A Matrix - System Dynamics (6×6):**

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
* **B Matrix - Control Input (6×2):**

$$
B = \begin{bmatrix}
0 & 0 \\
\frac{-sin(θ)}{m} & \frac{-sin(θ)}{m} \\
0 & 0 \\
\frac{cos(θ)}{m} & \frac{cos(θ)}{m} \\
0 & 0 \\
\frac{r}{I} & \frac{-r}{I}
\end{bmatrix}
$$

* **C Matrix - Measurement (3×6):**

$$
C = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

* **D Matrix  (3×2):**

$$
D = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0
\end{bmatrix}
$$

- $$\mathbf{D} = 0_{m \times n}$$, because control inputs don't directly appear in sensor readings.
The sensors only measure the drone's actual state, not the commands you're sending

#### Discrete-Time EKF Matrices
For the discrete EKF update, we use:
- **State transition Jacobian**: $F = I + A \Delta t$
- **Measurement Jacobian**: $H = C$ (constant)

### F and H matrices

$$F = I + A \Delta t$$, where I symbolizes the old state and A symbolizes the change of it.

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

The H matrix is the same as the C matrix, since they tell us what we can measure with our sensors:

$$
H = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$


### Noise Modeling
- **Process noise**: $\sigma = 0.004$ (applied to all state derivatives)
- **Measurement noise**: $\sigma = 0.05$ (on each sensor channel)

The simulation generates three trajectories:
- `state.clean`: Noise-free ground truth (nonlinear integration),
- `state.real`: "Real-world" with added process + sensor noise,
- `state.estimate`: EKF output (updated at every time step).


### Constant Terms

* **Gravity vector $G$:**  

$$
G = \begin{bmatrix}
0 \\
0 \\
0 \\
-g \\
0 \\
0
\end{bmatrix}
$$

### Key Linearization Results

**Partial derivatives:**  
- $\frac{\partial \ddot{x}}{\partial \theta} = -\frac{\cos\theta}{m}(u_1 + u_2)$
  
- $\frac{\partial \ddot{y}}{\partial \theta} = -\frac{\sin\theta}{m}(u_1 + u_2)$
  
- $\frac{\partial \ddot{x}}{\partial u_1} = -\frac{\sin\theta}{m}$, $\frac{\partial \ddot{x}}{\partial u_2} = -\frac{\sin\theta}{m}$
  
  
- $\frac{\partial \ddot{y}}{\partial u_1} = \frac{\cos\theta}{m}$, $\frac{\partial \ddot{y}}{\partial u_2} = \frac{\cos\theta}{m}$

- $\frac{\partial \ddot{\theta}}{\partial u_1} = \frac{r}{I}$, $\frac{\partial \ddot{\theta}}{\partial u_2} = -\frac{r}{I}$

## Extended Kalman Filter
### Formulas and Definitions
**Step 0: Initialization**

Kalman filter needs some initial data to begin working. We need:
- Initial conditions vector: $\ x̂_0$
- Initial variance matrix: $\ P_0 = I_{6x6} $
- Process noise variance matrix: $\ Q = 10^{-3}\\cdot \ I_{6x6} $
- Measurement noise variance matrix: $\ R = 10^{-3}\\cdot \ I_{6x6} $

**Step 1: Prediction**

Here we predict the state of the system and calculate the estimated variance matrix:

- $\ x̂_i^- = Fx̂_{i-1} $
- $\ P_{i}^- = FP_{i-1}F^T + Q $ 

**Step 2: Correction**

We have to calculate gain coefficient, which defines our trust of the new measurement; our state estimate at the new step; and an update ot the variance matrix:

- $\ k_i = P_i^-(P_{i-1} + R)^{-1} $
- $\ x̂_i = (I-k_i)x̂^-_i + k_iz_i $
- $\ P_i = (I - k_i)P^-_{i} $

**Step 3: Iteration**

This step is just a technicality for updating current state and variance matrix of the system:

- $\ x̂_{i-1}^- = x̂_{i} $
- $\ P_{i-1} = P_i $
  
# MATLAB Implementation Breakdown

The code is modularized into three key components:

## 1. Main Script (`Planar_Quadrotor.m`)

Initializes physical parameters and simulation settings (`dt = 0.01 s`, `T = 10 s`).  
Defines time-varying control inputs to excite the system:

```matlab
CU1 = 3 + 0.001*cos(2*time);
CU2 = 3 + 0.001*sin(2*time);
```
These represent small oscillations around hover thrust (~3 N), ensuring nontrivial dynamics
Calls simulation_quadrotor.m to run simulation + EKF.
Calls plot_quadrotor_results.m to generate final figures.
## 2. Simulation & EKF (simulation_quadrotor.m)
Performs three parallel simulations in one loop:
- state.clean: Noise-free ground truth via nonlinear integration of the true dynamics.
- state.real: "Real-world" trajectory with added process + sensor noise.
- state.estimate: Real-time EKF:
    - Prediction: Propagates state using the full nonlinear dynamics (not linear model).
    - Jacobian F: Computed at each step as $$F = I + A \Delta t$$, where $$A$$ is evaluated using the surrent estimates of \theta and control inputs $$u_1$$, $$u_2$$.

Noise tuning:
- Process noise covariance: $$Q=10^-5I_6$$ (high trust in physics model)
- Measurement noise covariance: $$R=10^-1I_3$$
## 3. Plotting Function (plot_quadrotor_results.m)
Generates two figures showing ground truth, noisy measurements, and EFK estinates together for all measured states.
### Simulation setup
It utilizes the:  - initial state hovering at 1 metre altitude
                  - Process noise.
                  - Measurement noise
                  
## Results & Visualization
### Figure 1: Measured States — Ground Truth vs Noisy vs EKF
![Measured States](Measured_state.png)

> The EKF (green) accurately tracks ground truth (blue) despite significant sensor noise (red). Vertical position ($y$), pitch angle ($\theta$), and angular rate ($\dot{\theta}$) are all well-estimated, confirming successful fusion of dynamics and measurements.

### Figure 2: Full State Estimation — Ground Truth vs EKF
![Full State](Full_state.png)

> Even **unmeasured states** like horizontal position ($x$) and velocities ($\dot{x}, \dot{y}$) are reconstructed accurately. This is possible due to **dynamic coupling**: thrust affects both $y$ and $\theta$, which indirectly informs $x$-motion through $\sin\theta$ and $\cos\theta$ terms.

