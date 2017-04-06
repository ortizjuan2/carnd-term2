#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    previous_timestamp_ = 0;

    is_initialized_ = false;

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 3.0;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.1;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.015;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.015;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.04;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.09;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.1;

    /**
TODO:

Complete the initialization. See ukf.h for other member properties.

Hint: one or more values initialized above might be wildly off...
*/

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

   // P_.fill(0.);

    P_ << 10, 0, 0, 0, 0,
    		0, 10, 0, 0, 0,
			0, 0, 10, 0, 0,
			0, 0, 0, 10, 0,
			0, 0, 0, 0, 10;



    n_x_ = 5;

    ///* Augmented state dimension
    n_aug_ = 7;

    lambda_ = 3 - n_aug_;

    Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    weights_ = VectorXd(2*n_aug_+1);

    n_z_radar_ = 3;

    n_z_laser_ = 2;

    // set weights

    weights_(0) = lambda_/(lambda_+n_aug_);
    for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }


    //add measurement noise covariance matrix
    R_radar_ = MatrixXd(n_z_radar_ ,n_z_radar_);
    R_radar_ <<    std_radr_*std_radr_, 0, 0,
             0, std_radphi_*std_radphi_, 0,
             0, 0,std_radrd_*std_radrd_;

    //add measurement noise covariance matrix
    R_laser_ = MatrixXd(n_z_laser_ ,n_z_laser_);
    R_laser_ <<    std_laspx_*std_laspx_, 0,
             0, std_laspy_*std_laspy_;



    Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    z_pred_radar_ = VectorXd(n_z_radar_);

    S_pred_radar_ = MatrixXd(n_z_radar_,n_z_radar_);

    z_pred_lidar_ = VectorXd(n_z_laser_);

    S_pred_lidar_ = MatrixXd(n_z_laser_,n_z_laser_);

    //create matrix for sigma points in measurement space
    Zsig_lidar_ = MatrixXd(n_z_laser_, 2 * n_aug_ + 1);

    //create matrix for sigma points in measurement space
    Zsig_radar_ = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
    /**
TODO:

Complete this function! Make sure you switch between lidar and radar
measurements.
*/

    // use first measurement to initialize state
    if(!is_initialized_){
        if(measurement_pack.raw_measurements_[0] != 0 && measurement_pack.raw_measurements_[1] != 0){
            double py, px;

            if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
                /**
                  Convert radar from polar to cartesian coordinates and initialize state.
                  */
                px = cos(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];
                py = sin(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];
                //x_ << px, py, measurement_pack.raw_measurements_[2], 0, 0;
                x_ << px, py, 1, measurement_pack.raw_measurements_[1], 0;

            }
            else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
                /**
                  Initialize state.
                  */
                px = measurement_pack.raw_measurements_[0];
                py = measurement_pack.raw_measurements_[1];
                x_ << px, py, 1, 0.1, 0;
            }

            is_initialized_ = true;
        }

        previous_timestamp_ = measurement_pack.timestamp_;

        return;
    }

    ///* Calculate delta t
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;


    AugmentedSigmaPoints();
    SigmaPointPrediction(dt);
    PredictMeanAndCovariance();

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        PredictRadarMeasurement();
        UpdateStateRadar(measurement_pack);

    }else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        PredictLidarMeasurement();
        UpdateStateLidar(measurement_pack);
    }


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
TODO:

Complete this function! Estimate the object's location. Modify the state
vector, x_. Predict sigma points, the state, and the state covariance matrix.
*/
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
TODO:

Complete this function! Use lidar data to update the belief about the object's
position. Modify the state vector, x_, and covariance, P_.

You'll also need to calculate the lidar NIS.
*/
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
TODO:

Complete this function! Use radar data to update the belief about the object's
position. Modify the state vector, x_, and covariance, P_.

You'll also need to calculate the radar NIS.
*/
}




void UKF::AugmentedSigmaPoints(void) {

    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    std::cout << "P_" << std::endl << P_ << std::endl << "----------------------" << std::endl;
    std::cout << "P_aug" << std::endl << P_aug << std::endl << "----------------------" << std::endl;
    std::cout << L << std::endl << "----------------------" << std::endl;

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++)
    {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }

    //write result
    Xsig_aug_ = Xsig_aug;

}



void UKF::SigmaPointPrediction(double delta_t) {

    //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

    //predict sigma points
    for (int i = 0; i< 2*n_aug_+1; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug_(0,i);
        double p_y = Xsig_aug_(1,i);
        double v = Xsig_aug_(2,i);
        double yaw = Xsig_aug_(3,i);
        double yawd = Xsig_aug_(4,i);
        double nu_a = Xsig_aug_(5,i);
        double nu_yawdd = Xsig_aug_(6,i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        Xsig_pred(0,i) = px_p;
        Xsig_pred(1,i) = py_p;
        Xsig_pred(2,i) = v_p;
        Xsig_pred(3,i) = yaw_p;
        Xsig_pred(4,i) = yawd_p;
    }

    //write result
    Xsig_pred_ = Xsig_pred;

}



void UKF::PredictMeanAndCovariance(void) {



    //create vector for predicted state
    VectorXd x = VectorXd(n_x_);

    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x_, n_x_);

    //predicted state mean
    x.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x = x+ weights_(i) * Xsig_pred_.col(i);
    }

    //predicted state covariance matrix
    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x;
        //angle normalization
        //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        x_diff(3) = constrainAngle(x_diff(3));

        P = P + weights_(i) * x_diff * x_diff.transpose() ;
    }
    //write result
    x_p_ = x;
    P_p_ = P;
}



void UKF::PredictRadarMeasurement(void) {

    //set measurement dimension, radar can measure r, phi, and r_dot


    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);
        if(Zsig(0,i)<1e-6)
        	Zsig(2,i) = 0.;
        else
        	Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot


    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_radar_);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
       // while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
       // while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        z_diff(1) = constrainAngle(z_diff(1));

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }


    S = S + R_radar_;

    //write result
    Zsig_radar_ = Zsig;
    z_pred_radar_ = z_pred;
    S_pred_radar_ = S;
}


void UKF::PredictLidarMeasurement(void) {

    //set measurement dimension, radar can measure r, phi, and r_dot

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_laser_, 2 * n_aug_ + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);

        // measurement model
        Zsig(0,i) = p_x;
        Zsig(1,i) = p_y;

    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_laser_);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z_laser_,n_z_laser_);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }


    S = S + R_laser_;

    //write result
    //create matrix for sigma points in measurement space
    Zsig_lidar_ = Zsig;
    z_pred_lidar_ = z_pred;
    S_pred_lidar_ = S;
}



void UKF::UpdateStateRadar(MeasurementPackage z) {


    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig_radar_.col(i) - z_pred_radar_;
        //angle normalization
        //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        z_diff(1) = constrainAngle(z_diff(1));

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_p_;

        //angle normalization
        //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        x_diff(3) = constrainAngle(x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S_pred_radar_.inverse();

    //residual
    VectorXd z_diff = z.raw_measurements_ - z_pred_radar_;

    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    z_diff(1) = constrainAngle(z_diff(1));

    //update state mean and covariance matrix
    x_ = x_p_ + K * z_diff;

    P_ = P_p_ - K*S_pred_radar_*K.transpose();



}


void UKF::UpdateStateLidar(MeasurementPackage z) {


    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_laser_);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig_lidar_.col(i) - z_pred_lidar_;
        //angle normalization
        //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        z_diff(1) = constrainAngle(z_diff(1));

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_p_;
                //angle normalization
        //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        x_diff(3) = constrainAngle(x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S_pred_lidar_.inverse();

    //residual
    VectorXd z_diff = z.raw_measurements_ - z_pred_lidar_;

    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    z_diff(1) = constrainAngle(z_diff(1));

    //update state mean and covariance matrix
    x_ = x_p_ + K * z_diff;
    P_ = P_p_ - K*S_pred_lidar_*K.transpose();


}


double UKF::constrainAngle(double ang){
    ang = fmod(ang + M_PI, 2.0*M_PI);
    if (ang < 0)
        ang += 2.0*M_PI;
    return ang - M_PI;
}








