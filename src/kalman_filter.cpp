#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
    pi = 3.141592653589793; // PI
    pi_2 = 2.0*pi; // 2PI
}

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
   float px = x_(0);
   float py = x_(1);
   float vx = x_(2);
   float vy = x_(3);
   float rho = sqrt(px*px + py*py);
   float phi = atan2(py, px); // [-pi, pi]
   float rho_dot = (px*vx + py*vy)/rho;
   // The estimated measurement values from estimated states
   VectorXd z_est =  VectorXd(3);
   z_est << rho, phi, rho_dot;
   //
   MatrixXd S = H_ * P_ * (H_.transpose()) + R_;
   MatrixXd K = P_ * (H_.transpose()) * (S.inverse());
   //new estimate
   VectorXd y = z - z_est;
   y(1) = correct_angle_rad( y(1) );
   x_ += K * y;
   P_ -= K * H_ * P_;
}

// Angle correction
float KalmanFilter::correct_angle_rad(float angle_in){
    // Correct the angle to (-pi, pi]
    float angle_mod = fmod(angle_in, pi_2); // [0, 2pi)
    if (angle_mod > pi){
        angle_mod -= pi_2;
    }else if(angle_mod <= -pi){
        angle_mod += pi_2;
    }
    return angle_mod;
}
