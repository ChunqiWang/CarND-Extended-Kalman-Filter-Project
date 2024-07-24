#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

MatrixXd I_;

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size); 
}

void KalmanFilter::Predict() {
  x_ = F_ * x_ ;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  std::cout << "Update (Laser) " << z << std::endl;
  VectorXd y_ = z - H_ * x_;
  UpdateCal(y_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  std::cout << "Update (Radar) " << z << std::endl;
  double px, py, vx, vy;
  px = x_(0);
  py = x_(1);
  vx = x_(2);
  vy = x_(3);

  double rho = sqrt(px*px + py*py);
  double theta = atan2(py, px);
  if (rho < 0.0001) {
    px += 0.0001;
    py += 0.0001;
    rho = sqrt(px*px + py*py);
  } 
  double rho_dot = (px*vx + py*vy) / rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;
  VectorXd y_ = z - z_pred;

  while (y_(1) > M_PI) {
    y_(1) -= 2*M_PI;
  }
  while (y_(1) < -M_PI) {
    y_(1) += 2*M_PI;
  }
  UpdateCal(y_);
}

void KalmanFilter::UpdateCal(const VectorXd &y_) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y_);
  P_ = (I_ - K * H_) * P_;
}
