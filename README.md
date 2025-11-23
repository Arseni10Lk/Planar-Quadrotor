# Planar Quadrotor 
- Linearized State-Space Model

## How We Got These Matrices

We started with the quadrotor's real physics equations and simplified them for when the drone is flying level (angle $\theta = 0$). We used small-angle approximations and calculated partial derivatives to see how sensitive each equation is to changes in the state variables.

## System Definitions

### State Vector
$$
\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ y \\ \dot{y} \\ \theta \\ \dot{\theta} \end{bmatrix}
$$

### Input Vector
$$
\mathbf{u} = \begin{bmatrix} u_1 \\ u_2 \end{bmatrix}
$$

### Measurement Vector
$$
\mathbf{y} = \begin{bmatrix} y \\ \dot{\theta} \\ \ddot{x} \\ \ddot{y} \end{bmatrix}
$$

## System Matrices

### A Matrix
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

### B Matrix
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

### C Matrix
$$
C = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & -\frac{u_1+u_2}{m} & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

### D Matrix
$$
D = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0 \\
\frac{1}{m} & \frac{1}{m}
\end{bmatrix}
$$
