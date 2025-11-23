# Planar-Quadrotor
- Linearized State-Space Model
## System Matrices (Linearized at $\theta = 0$)

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

### A Matrix ($6 \times 6$)
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
$$
C = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & -\frac{u_1+u_2}{m} & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

### D Matrix ($4 \times 2$)
$$
D = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0 \\
\frac{1}{m} & \frac{1}{m}
\end{bmatrix}
$$

## Parameters
- $m$: mass
- $I$: moment of inertia  
- $r$: rotor distance from center
- $g$: gravity (constant term in implementation)
