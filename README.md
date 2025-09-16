# Kinematic Calibration

C++17 project to explore the concept of Kinematic Calibration using Non-linear Least Square algorithms on real-world data gathered from a KUKA robot.
Real World End-Effector position and Joint angles were obtained from here: [Kalibrot-KUKA](https://github.com/cursi36/Kalibrot/tree/master/RealRobotsData/KUKA_IIWA_LBR14).

All the classes implemented are templated and are meant to function with every open chain robot manipulator composed of revolute joints (although they were only tested with a 7 DOF robot manipulator).

The Dataset was split in training set (80%) and validation set (20%). The initial conditions of the DH parameters were taken from the paper [Kalibrot-IEEE](https://ieeexplore.ieee.org/abstract/document/9635859)

## Dependencies
- [ceres-solver 2.2.0](https://github.com/ceres-solver/ceres-solver)
- [Eigen 3.4.0](https://gitlab.com/libeigen/eigen)

## Theory
Given a generic open-chain robot manipulator, the coordinate transformation between link $i-1$ and link $i$ can be systematically obtained using Denavit-Hartenberg parameters ($a$, $\alpha$, $d$, $\theta$).

The resulting homogeneous transformation between link $i-1$ and $i$ can be obtained by concatenating two rotations and two translations which can be compactly written as:

$$A^{i-1}_i = 
\begin{bmatrix} c_{\theta_i} & -s_{\theta_i} & 0 & 0 \\\ s_{\theta_i} & c_{\theta_i} & 0 & 0 \\\ 0 & 0 & 1 & d_i \\\ 0 & 0 & 0 & 1 \end{bmatrix} 
\begin{bmatrix} 1 & 0 & 0 & a_i \\\ 0 & c_{\alpha_i} & -s_{\alpha_i} & 0 \\\ 0 & s_{\alpha_i} & c_{\alpha_i} & 0 \\\ 0 & 0 & 0 & 1 \end{bmatrix} = 
\begin{bmatrix} c_{\theta_i} & -s_{\theta_i}c_{\alpha_i} & s_{\theta_i}s_{\alpha_i} & a_ic_{\theta_i} \\\ s_{\theta_i} & c_{\theta_i}c_{\alpha_i} & -c_{\theta_i}s_{\alpha_i} & a_is_{\theta_i} \\\ 0 & s_{\alpha_i} & c_{\alpha_i} & d_i \\\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Concatenating the homogeneous transfoormations of each link yields the position and orientation of the end-effector:

$$A = \prod_0^e A_{i-1}^i$$

The Non-linear least square algorithm used requires the definition of the Jacobian of these transformations wrt the DH parameters. Let $n$ be the number of links of the robot manipulator and $r$ be the number of residuals, the Jacobian matrix is an $r \times 4n$ matrix.

Taking the expression above and differentiating it for the generic DH parameter $\zeta$ yields the following expression:

$$\frac{\delta A}{\delta \zeta_i} = A_0 A_1 \dot{}\dot{}\dot{} \frac{\delta A_i}{\delta \zeta_i} \dot{}\dot{}\dot{} A_{i+1} A_{i+2} \dot{}\dot{}\dot{} A_{e}$$

Since any homogeneous transformation $A_{j \ne i}$ does not depend on $\zeta_i$ its jacobian is a $4\times 4$ matrix of zeros, and it does not contribute to the overall jacobian.

For the case where $j = i$ the following Jacobians wrt the DH parameters were considered (for my sanity I have omitted the subscript):

$$\frac{\delta A}{\delta a} = \begin{bmatrix}0 & 0 & 0 & c_{\theta} \\\ 0 & 0 & 0 & s_{\theta} \\\ 0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 0\end{bmatrix}$$

$$\frac{\delta A}{\delta \alpha} = \begin{bmatrix}0 & s_{\theta}s_{\alpha} & s_{\theta}c_{\alpha} & 0\\\0 & -c_{\theta}s_{\alpha} & -c_{\theta}c_{\alpha} & 0\\\0 & c_{\alpha} & -s_{\alpha} & 0\\\0 & 0 & 0 & 0\end{bmatrix}$$

$$\frac{\delta A}{\delta d} = \begin{bmatrix}0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 1 \\\ 0 & 0 & 0 & 0\end{bmatrix}$$

$$\frac{\delta A}{\delta \theta} = \begin{bmatrix}-s_{\theta} & -c_{\theta}c_{\alpha} & c_{\theta}s_{\alpha} & -as_{\theta}\\\ c_{\theta} & -s_{\theta}c_{\alpha} & s_{\theta}s_{\alpha} & ac_{\theta} \\\ 0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 0\end{bmatrix}$$

## Results

```console
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  3.740019e+01    0.00e+00    1.94e+03   0.00e+00   0.00e+00  1.00e+04        0    1.69e-01    2.06e-01
   1  5.547126e-02    3.73e+01    4.12e+00   0.00e+00   1.00e+00  3.00e+04        1    5.39e-01    7.50e-01
   2  7.427544e-02   -1.88e-02    4.12e+00   1.18e-02  -1.98e+00  1.50e+04        1    1.55e+00    2.30e+00
   3  7.142058e-02   -1.59e-02    4.12e+00   1.11e-02  -1.68e+00  3.75e+03        1    1.65e+00    3.95e+00
   4  6.807647e-02   -1.26e-02    4.12e+00   1.04e-02  -1.33e+00  4.69e+02        1    1.58e+00    5.53e+00
   5  6.029644e-02   -4.83e-03    4.12e+00   7.99e-03  -5.32e-01  2.93e+01        1    1.84e+00    7.37e+00
   6  5.278812e-02    2.68e-03    1.07e+01   4.04e-03   4.09e-01  2.91e+01        1    4.92e-01    7.86e+00
   7  5.263951e-02    1.49e-04    5.07e+00   2.22e-03   4.58e-02  1.66e+01        1    6.09e-01    8.47e+00
   8  5.308137e-02   -4.42e-04    5.07e+00   1.43e-03  -2.55e-01  8.32e+00        1    1.78e+00    1.02e+01
   9  5.262127e-02    1.82e-05    4.38e+00   2.24e-04   1.42e-02  4.34e+00        1    5.95e-01    1.08e+01
  10  5.254021e-02    8.11e-05    1.67e+00   7.01e-04   8.80e-02  2.78e+00        1    6.03e-01    1.14e+01
  11  5.263217e-02   -9.20e-05    1.67e+00   3.53e-04  -1.79e-01  1.39e+00        1    1.64e+00    1.31e+01
  12  5.255426e-02   -1.40e-05    1.67e+00   2.57e-04  -3.69e-02  3.48e-01        1    1.62e+00    1.47e+01
  13  5.251101e-02    2.92e-05    1.18e+00   1.36e-04   1.51e-01  2.59e-01        1    5.94e-01    1.53e+01
  14  5.250700e-02    4.01e-06    1.16e+00   8.76e-05   3.09e-02  1.42e-01        1    4.84e-01    1.58e+01
  15  5.250670e-02    2.98e-07    1.15e+00   4.74e-05   3.98e-03  7.19e-02        1    4.62e-01    1.62e+01
  16  5.250687e-02   -1.67e-07    1.15e+00   2.43e-05  -4.21e-03  3.60e-02        1    1.50e+00    1.77e+01
  17  5.250657e-02    1.30e-07    1.15e+00   1.30e-05   6.20e-03  1.83e-02        1    5.97e-01    1.83e+01

Link         a [m]        alpha [rad]        d [m]        theta [rad]
   1      -0.002983        1.574114        0.351515        3.138440
   2      -0.001053        1.572035        0.000732        3.140892
   3       0.001879        1.570023        0.423318        0.003150
   4       0.000813        1.569453        0.000010        3.141651
   5      -0.001474        1.572458        0.402597        0.000000
   6      -0.000974        1.572486       -0.000218        3.133899
   7       0.001267        0.000000        0.126818        0.000000

Mean Squared Root: 0.0014202
```

