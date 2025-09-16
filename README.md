# Kinematic Calibration

C++17 project to explore the concept of Kinematic Calibration using Non-linear Least Square algorithms on real-world data gathered from a KUKA robot.

Real World End-Effector position and Joint angles were obtained from here: [Kalibrot-KUKA](https://github.com/cursi36/Kalibrot/tree/master/RealRobotsData/KUKA_IIWA_LBR14).

All the classes implemented are templated and are meant to function with every open chain robot manipulator composed of revolute joints (although they were only tested with a 7 DOF robot manipulator).

The Dataset was split in training set (80%) and validation set (20%). The initial conditions of the DH parameters were taken from the paper [Kalibrot-IEEE](https://ieeexplore.ieee.org/abstract/document/9635859)

## Dependencies
- [ceres-solver 2.2.0](https://github.com/ceres-solver/ceres-solver)
- [Eigen 3.4.0](https://gitlab.com/libeigen/eigen)

## Results

```console
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  3.740019e+01    0.00e+00    1.94e+03   0.00e+00   0.00e+00  1.00e+04        0    1.78e-01    2.16e-01
   1  5.585563e-02    3.73e+01    4.25e+00   0.00e+00   1.00e+00  3.00e+04        1    4.07e-01    6.23e-01
   2  7.200665e-02   -1.62e-02    4.25e+00   3.14e-01  -1.64e+00  1.50e+04        1    1.79e-01    8.02e-01
   3  7.028472e-02   -1.44e-02    4.25e+00   2.09e-01  -1.46e+00  3.75e+03        1    1.76e-01    9.78e-01
   4  6.745789e-02   -1.16e-02    4.25e+00   1.29e-01  -1.18e+00  4.69e+02        1    1.93e-01    1.17e+00
   5  5.957438e-02   -3.72e-03    4.25e+00   1.27e-01  -3.93e-01  2.93e+01        1    2.27e-01    1.40e+00
   6  5.129512e-02    4.56e-03    1.54e+00   1.85e-01   6.57e-01  3.02e+01        1    4.27e-01    1.83e+00
   7  5.211127e-02   -8.16e-04    1.54e+00   1.04e-02  -4.62e-01  1.51e+01        1    2.32e-01    2.06e+00
   8  5.155607e-02   -2.61e-04    1.54e+00   2.07e-02  -2.16e-01  3.78e+00        1    2.28e-01    2.29e+00
   9  5.119102e-02    1.04e-04    1.19e+00   2.30e-02   1.88e-01  3.04e+00        1    4.25e-01    2.71e+00
  10  5.126025e-02   -6.92e-05    1.19e+00   9.94e-03  -2.44e-01  1.52e+00        1    1.80e-01    2.89e+00
  11  5.121582e-02   -2.48e-05    1.19e+00   6.79e-03  -1.55e-01  3.80e-01        1    1.76e-01    3.07e+00
  12  5.119134e-02   -3.26e-07    1.19e+00   2.30e-03  -6.72e-03  4.75e-02        1    1.75e-01    3.24e+00
  13  5.119016e-02    8.60e-07    1.21e+00   3.71e-04   1.20e-01  3.30e-02        1    3.29e-01    3.57e+00
  14  5.118976e-02    3.99e-07    1.22e+00   2.42e-04   8.37e-02  2.09e-02        1    3.33e-01    3.90e+00
  15  5.118957e-02    1.89e-07    1.23e+00   1.48e-04   6.41e-02  1.26e-02        1    3.45e-01    4.25e+00
  16  5.118948e-02    9.34e-08    1.23e+00   8.74e-05   5.33e-02  7.35e-03        1    4.30e-01    4.68e+00


Link         a [m]        alpha [rad]        d [m]        theta [rad]
   1      -0.002565        1.573619        0.351557        3.138699
   2      -0.000713        1.571931        0.000826        3.141050
   3       0.001538        1.569848        0.423230        0.002787
   4       0.000693        1.569593       -0.000063        3.141666
   5      -0.001366        1.573008        0.402656       -0.006676
   6      -0.000702        1.571802       -0.000130        3.136042
   7       0.001005        0.000000        0.126804       -0.208238


Mean Squared Root: 0.00137537
```

