#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    //
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
              0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

    /**
    * TODO: Finish initializing the FusionEKF.
    * TODO: Set the process and measurement noises
    */
    // create a 4D state vector, we don't know yet the values of the x state
    Eigen::VectorXd x_in = VectorXd(4);

    // state covariance matrix P
    Eigen::MatrixXd P_in = MatrixXd(4, 4);
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    // the initial transition matrix F_
    Eigen::MatrixXd F_in = MatrixXd(4, 4);
    F_in << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;


    // set the acceleration noise components
    noise_ax = 9.0;
    noise_ay = 9.0;
    // process covariance matrix
    Eigen::MatrixXd Q_in = MatrixXd(4,4);
    float dt = 0.001; // 1ms
    float dt_2 = dt*dt;
    float dt_3 = dt_2*dt;
    float dt_4 = dt_3*dt;
    //
    float dt_3_2 = dt_3 * 0.5;
    float dt_4_4 = dt_4 * 0.25;
    Q_in << dt_4_4*noise_ax, 0, dt_3_2*noise_ax, 0,
            0,          dt_4_4*noise_ay, 0, dt_3_2*noise_ay,
            dt_3_2*noise_ax, 0,  dt_2*noise_ax, 0,
            0, dt_3_2*noise_ay,  0, dt_2*noise_ay;
    //
    ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
    * Initialization
    */
    if (!is_initialized_) {
        /**
         * TODO: Initialize the state ekf_.x_ with the first measurement.
         * TODO: Create the covariance matrix.
         * You'll need to convert radar from polar to cartesian coordinates.
         */

        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
          // TODO: Convert radar from polar to cartesian coordinates
          //         and initialize state.

        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // TODO: Initialize state.
            // set the state with the initial location and zero velocity
            ekf_.x_ << measurement_pack.raw_measurements_[0],
                      measurement_pack.raw_measurements_[1],
                      0,
                      0;
            ekf_.P_ << 1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1000, 0,
                      0, 0, 0, 1000;
        }

        // Update the last timestamp
        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }// end Initialization

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    /**
    * Prediction
    */

    /**
    * TODO: Update the state transition matrix F according to the new elapsed time.
    * Time is measured in seconds.
    * TODO: Update the process noise covariance matrix.
    * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    */
    // 1. Modify the F matrix so that the time is integrated
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;
    // 2. Set the process covariance matrix Q
    float dt_2 = dt*dt; // dt**2
    float dt_3 = dt_2*dt; // dt**3
    float dt_4 = dt_3*dt; // dt**4
    //
    float dt_3_2 = dt_3 * 0.5; // dt**3/2
    float dt_4_4 = dt_4 * 0.25; // dt**4/4
    //
    ekf_.Q_ = MatrixXd(4,4);
    ekf_.Q_ << dt_4_4*noise_ax, 0, dt_3_2*noise_ax, 0,
            0,          dt_4_4*noise_ay, 0, dt_3_2*noise_ay,
            dt_3_2*noise_ax, 0,  dt_2*noise_ax, 0,
            0, dt_3_2*noise_ay,  0, dt_2*noise_ay;
    // 3. Call the Kalman Filter predict() function
    ekf_.Predict();

    /**
    * Update
    */

    /**
    * TODO:
    * - Use the sensor type to perform the update step.
    * - Update the state and covariance matrices.
    */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

    } else {
        // TODO: Laser updates
        // Assign H_ and R_
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;

        // 4. Call the Kalman Filter update() function
        //      with the most recent raw measurements_
        ekf_.Update( measurement_pack.raw_measurements_ );
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
