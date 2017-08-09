#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// -----------------------------------------------------------------------------
// Calculate the RMSE
// -----------------------------------------------------------------------------
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // The estimation vector size should not be zero
  // The estimation vector size should equal ground truth vector size
  if (estimations.size() == 0 || (estimations.size() != ground_truth.size())) {
    cout << "estimation vector size is 0 or not matching ground_truth vector size\n" << endl;
    return rmse;
  }

  // Accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // Residual
    VectorXd res = estimations[i] - ground_truth[i];
   
    // Multiply element-wise and accumulate 
    //res = res.array() * res.array();
    //rmse += res;
    rmse = rmse.array() + res.array() * res.array();
  }

  // Calculate the mean
  rmse = rmse / estimations.size();

  // Calculate the squared root to get RMSE
  rmse = rmse.array().sqrt();

  return rmse;
}

// -----------------------------------------------------------------------------
// Calculate Jacobian
// -----------------------------------------------------------------------------
MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3,4);

  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Denominators of the elements of Hj
  float den1 = px * px + py * py;
  float den2 = sqrt(den1);
  float den3 = den1 * den2;

  // Check for division by zero
  if (fabs(den1) < 0.0001) {
    cout << "CalculateJacobian() - Error - Division by Zero" << endl;
    return Hj;
  }

  // Compute the Jacobian matrix
  Hj << (px / den2), (py / den2), 0, 0,
        (-py / den1), (px / den1), 0, 0,
        py * (vx * py - vy * px) / den3, px * (vy * px - vx * py) / den3, (px / den2), (py / den2);

  return Hj;
}

// -----------------------------------------------------------------------------
// Normalize angle in radians to the range [-pi, pi)
// -----------------------------------------------------------------------------
const float M_2PI = 2 * M_PI;
float Tools::NormalizeAngle1(float r) {
  r = fmod(r + M_PI, M_2PI);
  if (r < 0)
    r += M_2PI;
  return r - M_PI;
}

// -----------------------------------------------------------------------------
// Normalize angle in radians to the range [-pi, pi]
// -----------------------------------------------------------------------------
float Tools::NormalizeAngle2(float r) {
  float y = fmod(r + M_PI, M_2PI);
  if (y < 0)
    y += M_2PI;

  if (r > 0.0 && y == 0.0)
    return y + M_PI;
  else
    return y - M_PI;
}
