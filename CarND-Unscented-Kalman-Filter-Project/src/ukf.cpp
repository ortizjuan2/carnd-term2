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
    std_a_ = 3.1;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 1.5;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.01;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.01;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.04;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.9;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 2.0;

    /**
TODO:

Complete the initialization. See ukf.h for other member properties.

Hint: one or more values initialized above might be wildly off...
*/

    ///* set state dimensions
    n_x_ = 5;

    ///* Set augmentation dimension
    n_aug_ = 7;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

       P_ << 10, 0, 0, 0, 0,
       0, 10, 0, 0, 0,
       0, 0, 10, 0, 0,
       0, 0, 0, 1000, 0,
       0, 0, 0, 0, 1000;


    //P_.fill(0.);

    Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);

    Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

    lambda_ = 3 - n_aug_;

    weights_ = VectorXd(2*n_aug_+1);

    ///* create weigths

    weights_(0) = lambda_/(lambda_+n_aug_);
    for(int i=1; i<(2*n_aug_+1);i++){
        weights_(i) = 0.5/(n_aug_+lambda_);
    }


    x = VectorXd(n_x_);
    P = MatrixXd(n_x_,n_x_);


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
                x_ << px, py, 1, 0, 0;
            }

            //CreateSigmaPointsAug(0.01); // use default dt this first time
            //Prediction(0.01);



            //ekf_.x_ << px, py, 0, 0;
            is_initialized_ = true;
        }
        previous_timestamp_ = measurement_pack.timestamp_;
        return;
    }

    ///* Calculate delta t
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    /*
       std::cout << "x_" << std::endl;
       std::cout << x_ << std::endl;
       std::cout << "--------------" << std::endl;
       std::cout << "P_" << std::endl;
       std::cout << P_ << std::endl;
       std::cout << "--------------" << std::endl;

*/

    //if (dt > 1e-4){


    CreateSigmaPointsAug(dt);

    Prediction(dt);

    //std::cout << P << std::endl;

    //} //////////////////////************************


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        UpdateRadar(measurement_pack);
    }else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        UpdateLidar(measurement_pack);
    }

    //std::cout << "-------P_-------" << std::endl;
    //std::cout << P_ << std::endl;

}


void UKF::CreateSigmaPointsAug(double dt){

    //////************************************

    ///* Define augmented state vector
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.head(5) = x_;
    x_aug(5) = 0.;
    x_aug(6) = 0.;


    ///* Define Augmented Covariance matrix
    P_aug = MatrixXd(7,7);

    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_ * std_a_;
    P_aug(6,6) = std_yawdd_ * std_yawdd_;

    ///* Create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    ///* Create augmented sigma points
    Xsig_aug.col(0) = x_aug;

    for(int i=0; i<n_aug_; i++){
        Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_)*L.col(i);
    }

    //std::cout << Xsig_aug << std::endl;
    //std::cout << "------------------" << std::endl;

    ///* define matrix to store predicted sigma points
    ///* note it is not augmented, must be same same of vector state
    //Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

    ///* predict sigma points
    //std::cout << "HERE 1" << std::endl;

    for(int i=0; i<(2*n_aug_+ 1); i++){
        ///* extract values for better readability
        double px = Xsig_aug(0,i);
        double py = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        ///* predicted state values
        double px_p, py_p;

        ///* avoid division by zero
        if(fabs(yawd) > 1e-3){
            px_p = px + v/yawd * (sin(yaw + yawd*dt) - sin(yaw));
            py_p = py + v/yawd * (cos(yaw) - cos(yaw+yawd*dt));
        }else{
            px_p = px + v*dt*cos(yaw);
            py_p = py + v*dt*sin(yaw);
        }

        double vp = v; ///* constant velocity, so predicted v is equal v
        double yaw_p = yaw + yawd*dt;



        double yawd_p = yawd; ///* constant acceleration

        ///* add noise
        px_p += 0.5*nu_a*dt*dt*cos(yaw);
        py_p += 0.5*nu_a*dt*dt*sin(yaw);
        vp += nu_a*dt;
        yaw_p += 0.5*dt*dt*nu_yawdd;

        //TODO: Angle normalization
         // while(yaw_p>M_PI) yaw_p-=2.*M_PI;
         //while(yaw_p<-M_PI) yaw_p+=2.*M_PI;


        yawd_p += nu_yawdd*dt;

        ///* write predicted sigma points
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = vp;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;

    }
    /*
       std::cout << "Xsig_pred_" << std::endl;
       std::cout << Xsig_pred_ << std::endl;
       std::cout << "--------------" << std::endl;

*/
    /////////*********************************


}



/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.h
 */
void UKF::Prediction(double dt) {
    /**
TODO:

Complete this function! Estimate the object's location. Modify the state
vector, x_. Predict sigma points, the state, and the state covariance matrix.
*/




    ///* predict state mean and state covariance


    x.fill(0.0);
    P.fill(0.0);

    // calculate predicted mean
    for(int i=0; i<(2*n_aug_+1); i++){
        x = x + weights_(i)*Xsig_pred_.col(i);
    }

    //    std::cout << x << std::endl;

    //// Checkpoint


    // calculate predicted covariance

    for(int i=0; i<(2*n_aug_ + 1); i++){
        VectorXd x_diff = Xsig_pred_.col(i) - x;
        //std::cout << "Checkpoint 1" << std::endl;
        ///* angle normalization

        /*
           std::cout << "Xsig_pred[" << i << "]:" << std::endl;
           std::cout << Xsig_pred_.col(i) << std::endl;
           std::cout << "x:" << std::endl;
           std::cout << x << std::endl;
           std::cout << "x_diff:" << std::endl;
           std::cout << x_diff << std::endl;
           */
        while(x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
        while(x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        //std::cout << "---------end-----------" << std::endl;
        //std::cout << "Checkpoint 2" << std::endl;
        P = P + weights_(i) * x_diff * x_diff.transpose();
    }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
    /**
TODO:

Complete this function! Use lidar data to update the belief about the object's
position. Modify the state vector, x_, and covariance, P_.

You'll also need to calculate the lidar NIS.
*/

    ///* measurement prediction
    ///* Create matrix for sigma points in measurement space
    MatrixXd Zsig;
    ///* create Vector for predicted measeurement mean
    VectorXd z_pred;
    int n_z; // set measurement dimession Radar or Lidar
    ///* Measurement covariance matrix X
    MatrixXd S;



    ///* lidar
    ///* radar
    ///* transform sigma points into radar measurement space
    n_z = 2;
    Zsig = MatrixXd(n_z, 2*n_aug_+1);
    for(int i=0; i<(2*n_aug_+1); i++){
        // extract values
        double px = Xsig_pred_(0,i);
        double py = Xsig_pred_(1,i);

        // Lidar measurement model

        Zsig(0,i) = px;
        Zsig(1,i) = py;

    }

    z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for(int i=0; i<(2*n_aug_+1); i++){
        z_pred += weights_(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for(int i=0; i<(2*n_aug_+1); i++){
        //residual
        VectorXd z_diff = Zsig.col(i)-z_pred;

        // angle normalization
        //while(z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
        //while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S += weights_(i)*z_diff*z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_*std_laspx_, 0,
      0, std_laspy_*std_laspy_;


    S += R; // final S matrix

    ///* UKF Update

    // Create matrix for cross correlation
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    for(int i=0; i<(2*n_aug_+1); i++){
        //residual
        VectorXd z_diff = Zsig.col(i)-z_pred;

        //TODO: Angle normalization when radar measurement

        //state difference
        VectorXd x_diff = Xsig_pred_.col(i)-x;
        // angle normalization
        while(x_diff(3)>M_PI) x_diff(3) -= 2.*M_PI;
        while(x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;


        Tc += weights_(i)*x_diff*z_diff.transpose();

    }

    // Kalman gain K
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = measurement_pack.raw_measurements_ - z_pred;

    //update state mean and covariance matrix

    x_ = x + K * z_diff;
    P_ = P - K*S*K.transpose();
    //std::cout << "State Updated!" << std::endl;



}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {
    /**
TODO:

Complete this function! Use radar data to update the belief about the object's
position. Modify the state vector, x_, and covariance, P_.

You'll also need to calculate the radar NIS.
*/


    ///* measurement prediction
    ///* Create matrix for sigma points in measurement space
    MatrixXd Zsig;
    ///* create Vector for predicted measeurement mean
    VectorXd z_pred;
    int n_z; // set measurement dimession Radar or Lidar
    ///* Measurement covariance matrix X
    MatrixXd S;




    ///* radar
    ///* transform predicted sigma points into radar measurement space
    n_z = 3;
    Zsig = MatrixXd(n_z, 2*n_aug_+1);
    for(int i=0; i<(2*n_aug_+1); i++){
        // extract values
        double px = Xsig_pred_(0,i);
        double py = Xsig_pred_(1,i);
        double v = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // Radar measurement model

        if(px == 0 && py == 0){
        	Zsig(0,i) = 0;
        	Zsig(1,i) = 0;		// psi
        	Zsig(2,i) = 0; // rho_dot
        }else{
        	double c1 = sqrt(px*px + py*py);
        	Zsig(0,i) = c1;					// rho
        	Zsig(1,i) = atan2(py,px);		// psi
        	Zsig(2,i) = (px*v1 + py*v2)/c1; // rho_dot
        }

    }

    z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for(int i=0; i<(2*n_aug_+1); i++){
        z_pred += weights_(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for(int i=0; i<(2*n_aug_+1); i++){
        //residual
        VectorXd z_diff = Zsig.col(i)-z_pred;

        // angle normalization
        while(z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
        while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S += weights_(i)*z_diff*z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_*std_radr_, 0, 0,
      0, std_radphi_*std_radphi_, 0,
      0, 0, std_radrd_*std_radrd_;

    S += R; // final S matrix



    ///* UKF Update

    // Create matrix for cross correlation
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    for(int i=0; i<(2*n_aug_+1); i++){
        //residual
        VectorXd z_diff = Zsig.col(i)-z_pred;

        //TODO: Angle normalization when radar measurement

        while(z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
        while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;



        //state difference
        VectorXd x_diff = Xsig_pred_.col(i)-x;
        // angle normalization
        while(x_diff(3)>M_PI) x_diff(3) -= 2.*M_PI;
        while(x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;


        Tc += weights_(i)*x_diff*z_diff.transpose();

    }

    // Kalman gain K
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = measurement_pack.raw_measurements_ - z_pred;
    //TODO: Angle normalization when Radar measurement

    while(z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
    while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;



    //update state mean and covariance matrix

    x_ = x + K * z_diff;
    P_ = P - K*S*K.transpose();
    //std::cout << "State Updated!" << std::endl;
    //std::cout << x_ << std::endl;
    //std::cout << "......." << std::endl;
    //std::cout << P_ << std::endl;



}