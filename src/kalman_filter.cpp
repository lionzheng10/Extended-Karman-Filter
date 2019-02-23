#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;//update state
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;//update covariance matrix
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  //calculate h
  VectorXd h = VectorXd (3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float c = sqrt (px*px + py*py);
  //avoid Divide by Zero by adding a small
  if (c < 0.00001)
  {
    px+=0.001;
    py+=0.001;
    c = sqrt (px*px + py*py);
  }
  h << c, atan2(py,px), (px*vx+py*vy)/c ;
  
  VectorXd y = z - h;
  //normalize angle to (-Pi~Pi)
  if(y(1) > M_PI){
    y(1) = y(1) - 2 * M_PI;
  }
  if(y(1) < -M_PI){
    y(1) = y(1) + 2 * M_PI;
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;//measurement covariance matrix
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;//Kalman filter gain
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
