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

### State-Space Representation
The IMU measures total acceleration including gravity, so we explicitly subtract $g$ when processing the $\ddot{y}$ measurement. The standard linearized system of state-space equations:

$$
\begin{aligned}
\dot{\mathbf{x}} &= A\mathbf{x} + B\mathbf{u} + G_x \\
\mathbf{y} &= C\mathbf{x} + D\mathbf{u} + G_y
\end{aligned}
$$


Where G and Gx contain constant terms like gravity.

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
### Observation Vector (4 elements)
We assume that we have an IMU, which measures linear accelerations and angular velocity, and an altitude sensor, which measures altitude. However, accelarations are measured relative to the body cor

$$
\mathbf{y} = 
\begin{bmatrix}
y \\
\dot{\theta} \\
\ddot{x_b} \\
\ddot{y_b}
\end{bmatrix}
$$

### Linearized System Matrices

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

* **C Matrix - Measurement (4×6):**

$$
C = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & \frac{-cos(θ)(u₁+u₂)}{m} & 0 \\
0 & 0 & 0 & 0 & \frac{-sin(θ)(u₁+u₂)}{m} & 0
\end{bmatrix}
$$

* **D Matrix  (4×2):**

$$
D = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
\frac{-sin(θ)}{m} & \frac{-sin(θ)}{m} \\
\frac{cos(θ)}{m} & \frac{cos(θ)}{m}]
\end{bmatrix}
$$

### F and H matrices

$$F = I + A \Delta t$$, where I symbolizes the old state and A symbolizes the change of it.

$$
F = \begin{bmatrix} 
1 & \Delta t & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & \frac{-cos(θ)(u₁+u₂)}{m} \Delta t & 0 \\
0 & 0 & 1 & \Delta t & 0 & 0 \\
0 & 1 & 0 & 0 & \frac{-sin(θ)(u₁+u₂)}{m} \Delta t & 0 \\
0 & 0 & 0 & 0 & 1 & \Delta t \\
0 & 0 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

### Constant Terms

* **Gravity vector for x $G_x$:**  

$$
G_x = \begin{bmatrix}
0 \\
0 \\
0 \\
-g \\
0 \\
0
\end{bmatrix}
$$


* **Gravity vector for y $G_y$:**  

$$
G_y = \begin{bmatrix}
0 \\
0 \\
0 \\
-g
\end{bmatrix}
$$
### Key Linearization Results

**Partial derivatives:**  
- $\frac{\partial \ddot{x}}{\partial \theta} = -\frac{\cos\theta}{m}(u_1 + u_2)$
  
- $\frac{\partial \ddot{y}}{\partial \theta} = -\frac{\sin\theta}{m}(u_1 + u_2)$
  
- $\frac{\partial \ddot{x}}{\partial u_1} = -\frac{\sin\theta}{m}$, $\frac{\partial \ddot{x}}{\partial u_2} = -\frac{\sin\theta}{m}$
  
  
- $\frac{\partial \ddot{y}}{\partial u_1} = \frac{\cos\theta}{m}$, $\frac{\partial \ddot{y}}{\partial u_2} = \frac{\cos\theta}{m}$

- $\frac{\partial \ddot{\theta}}{\partial u_1} = \frac{r}{I}$, $\frac{\partial \ddot{\theta}}{\partial u_2} = -\frac{r}{I}$

