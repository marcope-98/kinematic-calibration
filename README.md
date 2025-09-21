# Kinematic Calibration

C++17 project to explore the concept of Kinematic Calibration using Non-linear Least Square algorithms on real-world data gathered from a KUKA robot.
Real World End-Effector position and Joint angles were obtained from here: [Kalibrot-KUKA](https://github.com/cursi36/Kalibrot/tree/master/RealRobotsData/KUKA_IIWA_LBR14).

All the classes implemented are templated and are meant to function with every open chain robot manipulator composed of revolute and prismatic joints.

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

$$\frac{\delta A}{\delta a} = \begin{bmatrix}0 & 0 & 0 & c_{\theta} \\\ 0 & 0 & 0 & s_{\theta} \\\ 0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 0\end{bmatrix}
\qquad
\frac{\delta A}{\delta \alpha} = \begin{bmatrix}0 & s_{\theta}s_{\alpha} & s_{\theta}c_{\alpha} & 0\\\0 & -c_{\theta}s_{\alpha} & -c_{\theta}c_{\alpha} & 0\\\0 & c_{\alpha} & -s_{\alpha} & 0\\\0 & 0 & 0 & 0\end{bmatrix}
\qquad
\frac{\delta A}{\delta d} = \begin{bmatrix}0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 1 \\\ 0 & 0 & 0 & 0\end{bmatrix}
\qquad
\frac{\delta A}{\delta \theta} = \begin{bmatrix}-s_{\theta} & -c_{\theta}c_{\alpha} & c_{\theta}s_{\alpha} & -as_{\theta}\\\ c_{\theta} & -s_{\theta}c_{\alpha} & s_{\theta}s_{\alpha} & ac_{\theta} \\\ 0 & 0 & 0 & 0 \\\ 0 & 0 & 0 & 0\end{bmatrix}$$

## Build

```bash
$ mkdir build && cd build
$ cmake ..
$ make
```

## Examples

Three examples are provided in the example folder:
- a 3 DOF planar robot composed of only revolute joints
- a Stanford Manipulator (6 DOF)
- a KUKA Robot

Once the examples are built one can execute them by running the following shell command:
```bash
$ cd build
$ ./examples/3R       # Runs the 3 DOF planar robot
$ ...
$ ./examples/Stanford # Runs the Stanford manipulator
$ ...
$ ./examples/KUKA     # Runs the KUKA robot
```


## Results

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

