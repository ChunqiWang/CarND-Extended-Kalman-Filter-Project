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
   * Initializing the FusionEKF.
   * Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  VectorXd x_ = VectorXd(4);
  x_ << 1, 1, 1, 1;

  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  MatrixXd P_ = MatrixXd::Zero(4, 4);
  MatrixXd Q_ = MatrixXd::Zero(4, 4);

  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
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
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double rho = measurement_pack.raw_measurements_(0);
      double theta = measurement_pack.raw_measurements_(1);
      double rho_dot =  measurement_pack.raw_measurements_(2);

      double px = cos(theta) * rho;
      double py = sin(theta) * rho;
      double vx = cos(theta) * rho_dot;
      double vy = sin(theta) * rho_dot;

      ekf_.x_ << px, py, vx, vy;
      // Confident about radar speed   
      ekf_.P_ <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1; 
   
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      double px = measurement_pack.raw_measurements_(0);
      double py = measurement_pack.raw_measurements_(1);

      ekf_.x_ << px, py, 0, 0;
      // Not sure on Laser speed
      ekf_.P_ <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1000, 0,
                  0, 0, 0, 1000;      
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  float noise_ax = 9;
  float noise_ay = 9;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set the process covariance matrix Q

  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  ekf_.Predict();
  cout << "P_ = " << ekf_.P_ << endl;

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	ekf_.R_ = R_radar_;
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
  	ekf_.Update(measurement_pack.raw_measurements_);

  }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
