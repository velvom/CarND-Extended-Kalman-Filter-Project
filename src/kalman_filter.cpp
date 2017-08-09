#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;    // State vector (our belief, predicted)
  P_ = P_in;    // State covariance matrix
  F_ = F_in;    // State Tranistion matrix
  H_ = H_in;    // Measurement matrix (projection of our belief of the object's current state)
  R_ = R_in;    // Measurement covariance matrix (uncertainty in sensor measurement)
  Q_ = Q_in;    // Process covariance matrix (uncertainty in acceleration noise)
}

// -----------------------------------------------------------------------------
// Predict using linear model for both LIDAR and RADAR
// -----------------------------------------------------------------------------
void KalmanFilter::Predict() {
  // Object's new (mean) state
  x_ = F_ * x_;
  // Object's new state covariance matrix
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

// -----------------------------------------------------------------------------
// Measurement Update Step for Linear Motion Model (Lidar)
// Update the state by using Standard Kalman Filter equations
// @param z - sensor measurement vector containing position
// -----------------------------------------------------------------------------
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;    // diff in position between measurement and prediction: 2x1
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;   // Kalman gain: 4x2
  //cout << "K = " << K.rows() << "x" << K.cols() << endl;

  // New estimate
  x_ = x_ + (K * y);    // Correction to prediction using latest measurement
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;    // Correction to P using latest measurement
}

// -----------------------------------------------------------------------------
// Measurement Update Step for Non-linear Motion Model (Radar)
// Update the state by using Extended Kalman Filter equations
// The EKF equations linearize the non-linearity using Jacobian matrix
// @param z - sensor measurement vector containing (rho, phi, rho_dot).transpose
//            in polar coordinates: z_radar
// -----------------------------------------------------------------------------
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Cartesian to polar coordinate mapping
  float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  float rho_dot = 0.0;
  // Check for division by zero
  if (fabs(rho) >= 0.0001) {
    rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
  }
  float phi = atan2(x_(1), x_(0));	// [-pi, pi]

  VectorXd z_pred(3); 
  z_pred << rho, phi, rho_dot;	// h(x_): maps from cartesian to polar coordinates
  VectorXd y = z - z_pred;    // diff in position between measurement and prediction: 3x1

  y(1) = Tools::NormalizeAngle1(y(1));	// Normalize the resulting phi to [-pi, pi)

  // Here H_ = Hj = Jacobian matrix
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;   // Kalman gain: 4x3
  //cout << "K = " << K.rows() << "x" << K.cols() << endl;

  // New estimate
  x_ = x_ + (K * y);    // Correction to prediction using latest measurement
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;    // Correction to P using latest measurement
}
