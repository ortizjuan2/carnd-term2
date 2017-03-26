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
      * Create the covariance matrix.
     **/
    // first measurement
    cout << "EKF: " << endl;
    //ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    VectorXd x_in = VectorXd(4);
	MatrixXd P_in = MatrixXd(4,4);
	MatrixXd F_in = MatrixXd(4,4);
	MatrixXd H_in = MatrixXd(2,4);
	MatrixXd R_in = MatrixXd(2,2);
	MatrixXd Q_in = MatrixXd(4,4);
	float py, px;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	px = cos(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];
    	py = sin(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];

    	//ekf_.x_ << px, py, 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	px = measurement_pack.raw_measurements_[0];
    	py = measurement_pack.raw_measurements_[1];
    }

    // initial vector state
    x_in << px, py, 0, 0;

    //state covariance matrix P
    P_in << 1, 0, 0, 0,
    		  0, 1, 0, 0,
    		  0, 0, 1000, 0,
    		  0, 0, 0, 1000;

  	//measurement matrix
    H_in << 1, 0, 0, 0,
    	  0, 1, 0, 0;

    //the initial transition matrix F_
    F_in << 1, 0, 1, 0,
    			  0, 1, 0, 1,
    			  0, 0, 1, 0,
    			  0, 0, 0, 1;

    Q_in <<  9, 0, 0, 0,
    		 0, 9, 0, 0,
    		 0, 0, 9, 0,
			 0, 0, 0, 9;


    ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);


    // done initializing, no need to predict or update
    is_initialized_ = true;
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

  	const float noise_ax = 9;
  	const float noise_ay = 9;

	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;


	float dt_2 = dt * dt;
		float dt_3 = dt_2 * dt;
		float dt_4 = dt_3 * dt;

		//Modify the F matrix so that the time is integrated
		ekf_.F_(0, 2) = dt;
		ekf_.F_(1, 3) = dt;

		//set the process covariance matrix Q
		ekf_.Q_ = MatrixXd(4, 4);
		ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
				   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
				   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
				   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;



  ekf_.Predict();

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
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
