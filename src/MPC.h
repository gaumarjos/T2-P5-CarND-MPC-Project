#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// This value assumes the model presented in the classroom is used.
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Both the reference cross track and orientation errors are 0.

// double ref_v = 62.1371;
const double ref_v = 100;

// Timestep length and duration
const size_t N = 15;
const double dt = 0.05;
const unsigned int latency_steps = 2; // 0.1s / dt

// Weights of each cost component
const int w_cte = 100;
const int w_epsi = 1;
const int w_v = 1;
const int w_delta = 1;
const int w_a = 5;
const int w_ddelta = 600;
const int w_da = 1;

struct ExtendedState {
  vector<double> x;
  vector<double> y;
  //vector<double> psi;
  //vector<double> v;
  //vector<double> cte;
  //vector<double> epsi;
  vector<double> delta;
  vector<double> a;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  ExtendedState Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
};

#endif /* MPC_H */
