#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
    * TODO: predict the state
    */
    x_ = F_ * x_;
    P_ = F_ * P_ * ( F_.transpose() ) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    * TODO: update the state by using Kalman Filter equations
    */
    VectorXd z_est =  H_ * x_;
    MatrixXd S = H_ * P_ * (H_.transpose()) + R_;
    MatrixXd K = P_ * (H_.transpose()) * (S.inverse());
    //new estimate
    x_ += K * (z - z_est);
    P_ -= K * H_ * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);
   float rho = sqrt(px*px + py*py);
   float phi = atan2(py, px);
   float rho_dot = (px*vx + py*vy)/rho;
   // The estimated measurement values from estimated states
   VectorXd z_est =  VectorXd(3);
   z_est << rho, phi, rho_dot;
   //
   MatrixXd S = H_ * P_ * (H_.transpose()) + R_;
   MatrixXd K = P_ * (H_.transpose()) * (S.inverse());
   //new estimate
   x_ += K * (z - z_est);
   P_ -= K * H_ * P_;
}
