# Kinematic Calibration

C++17 project to explore the concept of Kinematic Calibration using Non-linear Least Squares algorithms on real-world data gathered from a KUKA robot.
Real World End-Effector positions and joint angles were obtained from here: [Kalibrot-KUKA](https://github.com/cursi36/Kalibrot/tree/master/RealRobotsData/KUKA_IIWA_LBR14).

All the classes implemented are templated and are meant to function with any open-chain robot manipulator composed of revolute and prismatic joints.

The dataset was split in training set (80%) and validation set (20%). The initial conditions of the DH parameters were taken from the paper [Kalibrot-IEEE](https://ieeexplore.ieee.org/abstract/document/9635859)

## Dependencies
- [ceres-solver 2.2.0](https://github.com/ceres-solver/ceres-solver)
- [Eigen 3.4.0](https://gitlab.com/libeigen/eigen)

## Theory
### What is Kinematic Calibration
Kinematic calibration is the process by which the actual geometric parameters of a robot (its link lengths, joint offsets, twist angles, etc...) are estimated and corrected to reduce the difference between the modeled kinematics and the real-world behaviour of the robot. Even though many industrial robots are very repetable, their absolute accuracy can be poor if the model assumes ideal geometry. By calibrating, systematic deviations due to manufacturing tolerances, assembly errors, or wear can be identified and corrected.

### Mathematical Modeling: DH Parameters
The calibration relies on a precise mathematical description of the robot's kinematics. This project uses the Denavit-Hartenberg (DH) convention to represent the spatial relationships between consecutive robot links. For each joint/link $i$, four parameters fully define the transformation from link $i-1$ to $i$: link length ($a_i$), link twist ($\alpha_i$), link offset ($d_i$), and joint angle ($\theta_i$).

Using these parameters, the homogeneous transformation matrix between link $i-1$ and $i$ can be obtained systematically as the matrix product of two rotations and two translations, which can be compactly written as:

$$A^{i-1}_i = 
\begin{bmatrix} c_{\theta_i} & -s_{\theta_i} & 0 & 0 \\\ s_{\theta_i} & c_{\theta_i} & 0 & 0 \\\ 0 & 0 & 1 & d_i \\\ 0 & 0 & 0 & 1 \end{bmatrix} 
\begin{bmatrix} 1 & 0 & 0 & a_i \\\ 0 & c_{\alpha_i} & -s_{\alpha_i} & 0 \\\ 0 & s_{\alpha_i} & c_{\alpha_i} & 0 \\\ 0 & 0 & 0 & 1 \end{bmatrix} = 
\begin{bmatrix} c_{\theta_i} & -s_{\theta_i}c_{\alpha_i} & s_{\theta_i}s_{\alpha_i} & a_ic_{\theta_i} \\\ s_{\theta_i} & c_{\theta_i}c_{\alpha_i} & -c_{\theta_i}s_{\alpha_i} & a_is_{\theta_i} \\\ 0 & s_{\alpha_i} & c_{\alpha_i} & d_i \\\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Concatenating the homogeneous transformations from the base to the end-effector gives the complete forward kinematic map:

$$A = A_0^1 A_2^1 \dot{}\dot{}\dot{} A_e^{e-1} = \prod_0^e A_{i-1}^i$$

This representation allows the end-effector pose to be expressed in a compact and differentiable form with respect to the DH parameters, which is essential for optimization.

### Error Modeling and Jacobian
To calibrate, a residual is defined as the difference between measured end-effector poses and predicted poses from the forward kinematics model. A non-linear least squares problem is formulated to minimize the sum of squared residuals by adjusting the DH parameters. The Jacobian of the end-effector pose with respect to each DH parameter is used to guide the optimizer:

$$\frac{\delta A}{\delta \zeta_i} = A_0 A_1 \dot{}\dot{}\dot{} \frac{\delta A_i}{\delta \zeta_i} \dot{}\dot{}\dot{} A_{i+1} A_{i+2} \dot{}\dot{}\dot{} A_{e}$$

where $\zeta_i$ represents one of the DH parameters ($a_i$, $\alpha_i$, $d_i$, $\theta_i$). Analytic computation of the Jacobian ensures efficient and accurate gradient evaluation, which is critical for convergence in non-linear optimization.

Since any homogeneous transformation $A_{j \ne i}$ does not depend on $\zeta_i$, its jacobian is a $4\times 4$ matrix of zeros and does not contribute to the overall jacobian.

For the case where $j = i$, the following Jacobians wrt the DH parameters were considered (for my sanity I have omitted the subscript):

$$\frac{\delta A}{\delta a} = \begin{bmatrix}0 & 0 & 0 & c_{\theta} \\\ 0 & 0 & 0 & s_{\theta} \\\ 0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 0\end{bmatrix}
\qquad
\frac{\delta A}{\delta \alpha} = \begin{bmatrix}0 & s_{\theta}s_{\alpha} & s_{\theta}c_{\alpha} & 0\\\0 & -c_{\theta}s_{\alpha} & -c_{\theta}c_{\alpha} & 0\\\0 & c_{\alpha} & -s_{\alpha} & 0\\\0 & 0 & 0 & 0\end{bmatrix}
\qquad
\frac{\delta A}{\delta d} = \begin{bmatrix}0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 1 \\\ 0 & 0 & 0 & 0\end{bmatrix}
\qquad
\frac{\delta A}{\delta \theta} = \begin{bmatrix}-s_{\theta} & -c_{\theta}c_{\alpha} & c_{\theta}s_{\alpha} & -as_{\theta}\\\ c_{\theta} & -s_{\theta}c_{\alpha} & s_{\theta}s_{\alpha} & ac_{\theta} \\\ 0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 0\end{bmatrix}$$

### Optimization Method
The calibration procedure is formulated as a nonlinear least-squares parameter estimation problem, where the objective is to identify the Denavit-Hartenberg parameters that best explain the measured end-effector poses obtained from the physical robot.

Given a set of joint configurations $q_k$ and the corresponding measured end-effector poses $y_k$, the optimization seeks the parameter vector $p$ that minimizes the sum of squared residuals:

$$\min_{p} \sum_{k=1}^{N} \left\lVert f(q_k, p) - y_k \right\rVert^2$$

where $f(q_k, p)$ is the forward kinematic map constructed from the DH parameters.

Because the forward kinematics model is nonlinear in the parameters, the resulting optimization problem is inherently nonlinear and must be solved with iterative numerical methods.

### Trust-Region Methods

The ceres-solver library uses trust-region optimization, tipically through the Levenber-Marquardt or Dogleg algorithms. These methods approximate the cost function locally by a quadratic model:

$$m(\Delta p) = \frac{1}{2} J^TJ\Delta p + J^T r$$

where $J$ is the Jacobian of the residual vector $r$. The update step $\Delta p$ is restricted to lie within a region where the quadratic model is considered reliable:

$$\left\lVert\Delta p \right\rVert \le \delta$$

After each iteration, the trust-region radius is adapted based on how accurately the quadratic model predicts the reduction in the cost function.

### Levenberg-Marquardt Algorithm

The Levenberg-Marquardt (LM) algorithm blends Gauss-Newton and gradient descent methods. The update rule is given by:

$$(J^T J + \lambda I) \Delta p = -J^T r$$

where small $\lambda$ gives Gauss-Netwon behaviour (fast near solution), large $\lambda$ gives gradient descent behaviour (safe when far from solution).

### Convergence Criteria

The solver iteratively updates the parameter vector until convergence, which is defined by one or more of the following criteria:
- Norm of the gradient is below a threshold
- Change in parameter vector $\Delta p$ is negligible
- Reduction in the cost function is sufficiently small

This ensures that the final DH parameters provide the best-fit model of the measured data.

## Build

```bash
$ mkdir build && cd build
$ cmake ..
$ make
```

## Examples

Three examples are provided in the `examples` folder, each with a different dataset:
- a 3-DOF planar robot composed only of revolute joints. Synthetic data was generated by a [script](https://github.com/cursi36/Kalibrot/blob/master/tests/getData_3R.m), where Gaussian noise with standard deviation of $1e-1$ meters is added to the ground truth end effector position. The ground truth values of the DH parameters for this manipulator are summarized in the following table:

<div align="center">
   
|joint| 1 | 2 | 3 |
|:-:|:-:|:-:|:-:|
|$d [m]$|0|0|0|
|$\theta [rad]$|0|0|0|
|$a [m]$|1|0.5|2|
|$\alpha [rad]$|0|0|0|

</div>

- a Stanford Manipulator (6 DOF). Synthetic data was generated by a [script](https://github.com/cursi36/Kalibrot/blob/master/tests/getData_Stanford.m), where Gaussian noise with standard deviation of $1e-1$ meters is added to the ground truth end effector position. The ground truth values of the DH parameters for this manipulator are summarized in the following table

<div align="center">
   
|joint| 1 | 2 | 3 | 4 | 5 | 6 |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|$d [m]$|1|1.5|0|0.5|0|0.1|
|$\theta [rad]$|$-\pi/2$|$\pi$|0|$-\pi/2$|$-\pi/2$|0|
|$a [m]$|0|0|0|0|0|0|
|$\alpha [rad]$|$-\pi/2$|$-\pi/2$|0|$\pi/2$|$-\pi/2$|0|

</div>

- a KUKA Robot. [Real-world data](https://github.com/cursi36/Kalibrot/tree/master/RealRobotsData/KUKA_IIWA_LBR14) collected from a KUKA manipulator with measured joint angles and end-effector positions. The expected values of the DH parameters for this manipulator are summarized in the following table

<div align="center">
   
|joint| 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|$d [m]$|0.36|0|0.42|0|0.4|0|0.126|
|$\theta [rad]$|$\pi$|$\pi$|0|$\pi$|0|$\pi$|0|
|$a [m]$|0|0|0|0|0|0|0|
|$\alpha [rad]$|$\pi/2$|$\pi/2$|$\pi/2$|$\pi/2$|$\pi/2$|$\pi/2$|0|

</div>

Once the examples are built, they can be execute using the following shell commands:
```bash
$ cd build
$ ./examples/3R       # Runs the 3 DOF planar robot
$ ...
$ ./examples/Stanford # Runs the Stanford manipulator
$ ...
$ ./examples/KUKA     # Runs the KUKA robot
```

## Results

During optimization, the solver reports iteration, cost, gradient norm, and step size. Furthermore dataset splitting allows quantification of generalization performance. After calibration, residual errors between predicted and measured end-effector poses are minimized, increasing forward kinematics accuracy.

The following are the iteration results and the expected report visualized during the execution of the KUKA example:

```console
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  3.740019e+01    0.00e+00    1.94e+03   0.00e+00   0.00e+00  1.00e+04        0    1.71e-01    2.06e-01
   1  7.885131e-02    3.73e+01    5.10e+01   0.00e+00   9.99e-01  3.00e+04        1    4.36e-01    6.42e-01
   2  7.419116e-02    4.66e-03    4.94e+01   8.63e-03   1.51e-01  2.24e+04        1    3.71e-01    1.01e+00
   3  6.125451e-02    1.29e-02    3.56e+01   4.22e-03   4.92e-01  2.24e+04        1    3.68e-01    1.38e+00
   4  6.034070e-02    9.14e-04    2.75e+01   2.78e-03   6.84e-02  1.36e+04        1    3.69e-01    1.75e+00
   5  6.131995e-02   -9.79e-04    2.75e+01   1.38e-03  -7.86e-02  6.80e+03        1    1.67e+00    3.42e+00
   6  6.033860e-02    2.09e-06    2.75e+01   3.83e-05   1.68e-04  1.70e+03        1    5.55e-01    3.97e+00
   7  6.008160e-02    2.59e-04    2.89e+01   2.38e-04   2.08e-02  9.05e+02        1    5.05e-01    4.48e+00
   8  5.990374e-02    1.78e-04    3.01e+01   1.99e-04   1.46e-02  4.72e+02        1    5.09e-01    4.99e+00
   9  5.982880e-02    7.49e-05    3.38e+01   7.73e-04   6.24e-03  2.41e+02        1    4.61e-01    5.45e+00
  10  5.949181e-02    3.37e-04    3.37e+01   3.23e-04   2.83e-02  1.31e+02        1    4.67e-01    5.91e+00
  11  5.788088e-02    1.61e-03    3.13e+01   1.78e-04   1.39e-01  9.51e+01        1    4.67e-01    6.38e+00
  12  5.579312e-02    2.09e-03    2.77e+01   1.32e-04   2.10e-01  7.96e+01        1    4.15e-01    6.80e+00
  13  5.447028e-02    1.32e-03    2.52e+01   1.05e-04   1.69e-01  6.17e+01        1    3.62e-01    7.16e+00
  14  5.308523e-02    1.39e-03    2.22e+01   9.16e-05   2.13e-01  5.19e+01        1    3.61e-01    7.52e+00
  15  5.214417e-02    9.41e-04    2.00e+01   8.02e-05   1.85e-01  4.15e+01        1    3.62e-01    7.88e+00
  16  5.124178e-02    9.02e-04    1.75e+01   7.21e-05   2.18e-01  3.52e+01        1    4.03e-01    8.28e+00
  17  5.061496e-02    6.27e-04    1.56e+01   6.49e-05   1.95e-01  2.87e+01        1    4.75e-01    8.76e+00
  18  5.004414e-02    5.71e-04    1.36e+01   5.79e-05   2.22e-01  2.45e+01        1    4.74e-01    9.23e+00
  19  4.964395e-02    4.00e-04    1.21e+01   5.24e-05   2.01e-01  2.01e+01        1    4.75e-01    9.71e+00
  20  4.929238e-02    3.52e-04    1.05e+01   4.61e-05   2.23e-01  1.72e+01        1    3.69e-01    1.01e+01
  21  4.904430e-02    2.48e-04    9.23e+00   4.17e-05   2.04e-01  1.43e+01        1    3.68e-01    1.04e+01
  22  4.883269e-02    2.12e-04    7.99e+00   3.63e-05   2.22e-01  1.22e+01        1    3.68e-01    1.08e+01
  23  4.868228e-02    1.50e-04    6.97e+00   3.27e-05   2.06e-01  1.01e+01        1    4.05e-01    1.12e+01
  24  4.855754e-02    1.25e-04    6.00e+00   2.83e-05   2.19e-01  8.59e+00        1    4.75e-01    1.17e+01
  25  4.846804e-02    8.95e-05    5.20e+00   2.53e-05   2.04e-01  7.12e+00        1    4.75e-01    1.22e+01
  26  4.839595e-02    7.21e-05    4.44e+00   2.18e-05   2.12e-01  5.98e+00        1    4.76e-01    1.26e+01
  27  4.834371e-02    5.22e-05    3.81e+00   1.93e-05   2.00e-01  4.92e+00        1    3.71e-01    1.30e+01
  28  4.830292e-02    4.08e-05    3.22e+00   1.68e-05   2.02e-01  4.06e+00        1    3.68e-01    1.34e+01
  29  4.827315e-02    2.98e-05    2.72e+00   1.48e-05   1.91e-01  3.28e+00        1    3.71e-01    1.38e+01
  30  4.825065e-02    2.25e-05    2.27e+00   1.31e-05   1.87e-01  2.64e+00        1    3.70e-01    1.41e+01
  31  4.823429e-02    1.64e-05    1.88e+00   1.17e-05   1.76e-01  2.07e+00        1    4.69e-01    1.46e+01
  32  4.822235e-02    1.19e-05    1.54e+00   1.09e-05   1.66e-01  1.60e+00        1    4.78e-01    1.51e+01
  33  4.821387e-02    8.48e-06    1.24e+00   1.04e-05   1.53e-01  1.20e+00        1    4.70e-01    1.55e+01
  34  4.820797e-02    5.90e-06    9.88e-01   1.01e-05   1.39e-01  8.70e-01        1    4.10e-01    1.60e+01
  35  4.820400e-02    3.97e-06    7.74e-01   9.79e-06   1.23e-01  6.09e-01        1    3.62e-01    1.63e+01
  36  4.820144e-02    2.56e-06    5.99e-01   9.14e-06   1.06e-01  4.09e-01        1    3.63e-01    1.67e+01
  37  4.819986e-02    1.58e-06    4.62e-01   7.98e-06   9.07e-02  2.64e-01        1    3.62e-01    1.70e+01
  38  4.819894e-02    9.21e-07    3.60e-01   6.40e-06   7.66e-02  1.64e-01        1    4.49e-01    1.75e+01
  39  4.819843e-02    5.15e-07    2.89e-01   4.70e-06   6.50e-02  9.91e-02        1    4.67e-01    1.80e+01
  40  4.819815e-02    2.81e-07    2.41e-01   3.18e-06   5.61e-02  5.83e-02        1    4.71e-01    1.84e+01
  41  4.819800e-02    1.51e-07    2.13e-01   2.02e-06   4.99e-02  3.37e-02        1    4.26e-01    1.89e+01
  42  4.819791e-02    8.19e-08    1.96e-01   1.22e-06   4.58e-02  1.93e-02        1    3.62e-01    1.92e+01


Link         a [m]        alpha [rad]        d [m]        theta [rad]
   1       0.000000        1.570825        0.351221        3.139314
   2       0.000247        1.571767        0.003237        3.143851
   3       0.000117        1.566196        0.424028        0.000722
   4       0.000000        1.566009        0.000000        3.140520
   5       0.000339        1.585097        0.401980        0.000000
   6       0.000157        1.585616        0.000000        3.142853
   7       0.000000        0.000000        0.127032        0.000000


Mean Squared Root: 0.00133813
```

