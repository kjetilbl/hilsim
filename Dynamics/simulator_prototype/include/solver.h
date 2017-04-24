#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

class NumericalSolver {
  // Template is for Ode45, but usable for other solvers as well. TODO: Add
  // functionality for choice of solver.
public:
  NumericalSolver();
  ~NumericalSolver();
  void initializeSolver(double dt);

  long double epsilon, t, y, h, h_min, h_max;

  long double a21, a31, a32, a41, a42, a43, a51, a52, a53, a54, a61, a62, a63, a64,
  a65, a71, a72, a73, a74, a75, a76, c2, c3, c4, c5, c6, c7, b11, b12, b13,
  b14, b15, b16, b17, b21, b22, b23, b24, b25, b26, b27;
  Vector6d k1v, k2v, k3v, k4v, k5v, k6v, k7v, zv, sv, errv;
};

#endif