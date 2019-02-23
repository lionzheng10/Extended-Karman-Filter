#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    VectorXd x_in = VectorXd(4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
			float rho = measurement_pack.raw_measurements_(0);
			float phi = measurement_pack.raw_measurements_(1);
			float rho_dot = measurement_pack.raw_measurements_(2);

			float px = rho * cos(phi);
			float py = rho * sin(phi);
			float vx = rho_dot * cos(phi);
			float vy = rho_dot * sin(phi);

			ekf_.x_ << px, py, vx, vy;
			previous_timestamp_ = measurement_pack.timestamp_;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
			ekf_.x_ << measurement_pack.raw_measurements_(0),measurement_pack.raw_measurements_(1),0,0;
			previous_timestamp_ = measurement_pack.timestamp_;
    }

		// state covariance matrix P
		MatrixXd P_in = MatrixXd (4,4);
		P_in << 1, 0, 0, 0,
		        0, 1, 0, 0,
		        0, 0, 1, 0,
		        0, 0, 0, 1;

		// state transition matrix F
		MatrixXd F_in = MatrixXd (4,4);
		F_in << 1, 0, 0, 0,
		        0, 1, 0, 0,
		        0, 0, 1, 0,
		        0, 0, 0, 1;

		// Measurement matrix H
		MatrixXd H_in = MatrixXd (2,2);
		H_in << 0.0225, 0,
		        0, 0.0225;

    // Measurement covariance matrix R
		MatrixXd R_in = MatrixXd (2,2);
    R_in << 0.0225, 0,
            0, 0.0225;

		// Process covariace matrix Q
		MatrixXd Q_in = MatrixXd (4,4);
		Q_in << 1, 0, 1, 0,
						0, 1, 0, 1,
						1, 0, 1, 0,
						0, 1, 0, 1;

		cout << "EKF Init: " << endl;
		ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "Init DONE: " << endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	noise_ax = 9;
	noise_ay = 9;

	ekf_.F_(0,2) = dt;
	ekf_.F_(1,3) = dt;

	ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
	           0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
	           dt_3/2*noise_ax, 0,dt_2*noise_ax, 0,
	           0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  cout << "EKF Predict: " << endl;
	ekf_.Predict();
	
	//  print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
		cout << "EKF Update (Radar): " << endl;
		ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.R_ = R_radar_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
		cout << "EKF Update (Laser):" << endl;
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
