#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse /= float(estimations.size());
  // calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE
  float rho_2 = (px*px + py*py);
  // check division by zero
  if ( fabs(rho_2) < 0.0001){
      std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
      return Hj;
  }
  // compute the Jacobian matrix
  float vxpy_vypx = vx*py - vy*px;
  float vypx_vxpy = -vxpy_vypx; // vy*px - vx*py; // -vxpy_vypx, Note: -0 will be consider "different" with 0 in checker
  float inv_rho_2 = 1.0/rho_2;
  float inv_rho = sqrt(inv_rho_2);
  float inv_rho_3 = inv_rho_2 * inv_rho;
  // CAlculate the Jacobian Hj
  Hj << inv_rho*px, inv_rho*py, 0, 0,
        -inv_rho_2*py, inv_rho_2*px, 0, 0,
        inv_rho_3*py*vxpy_vypx, inv_rho_3*px*vypx_vxpy, inv_rho*px, inv_rho*py;
  return Hj;
}
