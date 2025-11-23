# Planar Quadrotor 
- Linearized State-Space Model

## Linearization Process

The system was linearized around the hover condition ($\theta = 0$) using the small-angle approximation ($\sin\theta \approx \theta$, $\cos\theta \approx 1$). The Jacobian matrices were derived by taking partial derivatives of the nonlinear equations with respect to each state variable.

### Original Nonlinear Equations
$$
\begin{aligned}
m\ddot{x} &= -(u_1 + u_2)\sin\theta \\
m\ddot{y} &= (u_1 + u_2)\cos\theta - mg \\
I\ddot{\theta} &= r(u_1 - u_2)
\end{aligned}
$$

### Linearized Equations
$$
\begin{aligned}
\ddot{x} &\approx -\frac{u_1 + u_2}{m}\theta \\
\ddot{y} &\approx \frac{u_1 + u_2}{m} - g \\
\ddot{\theta} &= \frac{r}{I}(u_1 - u_2)
\end{aligned}
$$

## System Matrices (Linearized at $\theta = 0$)

### State Vector
$$
\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ y \\ \dot{y} \\ \theta \\ \dot{\theta} \end{bmatrix}
$$

### Input Vector
$$
\mathbf{u} = \begin{bmatrix} u_1 \\ u__2 \end{bmatrix}
$$

### Measurement Vector
$$
\mathbf{y} = \begin{bmatrix} y \\ \dot{\theta} \\ \ddot{x} \\ \ddot{y} \end{bmatrix}
$$

### A Matrix ($6 \times 6$)
**Derivation**: Combines trivial kinematic relationships ($\dot{x} = \dot{x}$, etc.) with linearized acceleration terms from partial derivatives.

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -\frac{u_1+u_2}{m} & 0 \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

### B Matrix ($6 \times 2$)
**Derivation**: From partial derivatives of equations with respect to control inputs $u_1$ and $u_2$.

$$
B = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0 \\
\frac{1}{m} & \frac{1}{m} \\
0 & 0 \\
\frac{r}{I} & -\frac{r}{I}
\end{bmatrix}
$$

### C Matrix ($4 \times 6$)
**Derivation**: Maps states to measurements based on sensor capabilities.

$$
C = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & -\frac{u_1+u_2}{m} & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

### D Matrix ($4 \times 2$)
**Derivation**: Accounts for direct effect of control inputs on measurements.

$$
D = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0 \\
\frac{1}{m} & \frac{1}{m}
\end{bmatrix}
$$

## Key Partial Derivatives

- $\frac{\partial \ddot{x}}{\partial \theta} = -\frac{u_1 + u_2}{m}$ (Tilting creates horizontal acceleration)
- $\frac{\partial \ddot{y}}{\partial u_1} = \frac{1}{m}$, $\frac{\partial \ddot{y}}{\partial u_2} = \frac{1}{m}$ (Rotors create vertical acceleration)
- $\frac{\partial \ddot{\theta}}{\partial u_1} = \frac{r}{I}$, $\frac{\partial \ddot{\theta}}{\partial u_2} = -\frac{r}{I}$ (Differential thrust creates rotation)

## Parameters
- $m$: mass
- $I$: moment of inertia  
- $r$: rotor distance from center
- $g$: gravity (handled as constant in implementation)

## Notes
- The $A$ and $C$ matrices depend on $(u_1+u_2)$, making this a Linear Parameter-Varying (LPV) system
- For EKF implementation, these matrices are recalculated at each time step using current state estimates
- Gravity term appears as constant offset in $\ddot{y}$ measurement