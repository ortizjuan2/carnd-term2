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
    std_yawdd_ = 0.3;//0.4

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.57;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.44;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.44;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.004;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.24;

    // set state dimensions
    n_x_ = 5;

    // Set augmentation state dimension
    n_aug_ = 7;

    // initial state vector
    x_ = VectorXd(5);

    x_.fill(0.0);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    P_ << 4, 0, 0, 0, 0,
       0, 3, 0, 0, 0,
       0, 0, 1, 0, 0,
       0, 0, 0, 1, 0,
       0, 0, 0, 0, 1;

    // Matrix for augmented sigman points
    Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);

    // Matrix for predicted sigmad points
    Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

    // define lambda parameter
    lambda_ = 3 - n_aug_;

    // weights in sigma points and prediction generation
    weights_ = VectorXd(2*n_aug_+1);

    // create weigths
    weights_(0) = lambda_/(lambda_+n_aug_);
    for(int i=1; i<(2*n_aug_+1);i++){
        weights_(i) = 0.5/(n_aug_+lambda_);
    }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {

    // use first measurement to initialize state
    if(!is_initialized_){
        if(measurement_pack.raw_measurements_[0] != 0 && measurement_pack.raw_measurements_[1] != 0){
            double py, px, v, vx, vy, psi;

            if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
                //Convert radar from polar to cartesian coordinates and initialize state.
                px = cos(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];
                py = sin(measurement_pack.raw_measurements_[1])*measurement_pack.raw_measurements_[0];
                v = fabs(measurement_pack.raw_measurements_[2]);
                vx = cos(measurement_pack.raw_measurements_[1])*v;
                vy = sin(measurement_pack.raw_measurements_[1])*v;
                psi = atan2(vy, vx);
                // initialize state
                x_ << px, py, v, psi, 0;
            }
            else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
                //Laser values are directed mapped to px and py
                px = measurement_pack.raw_measurements_[0];
                py = measurement_pack.raw_measurements_[1];
                // initialize state
                x_ << px, py, 0, 0, 0;
            }
            is_initialized_ = true;
        }
        previous_timestamp_ = measurement_pack.timestamp_;
        return;
    }

    // Calculate delta t
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    // start a prediction
    Prediction(dt);
    // update state according to type of measurement received
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        UpdateRadar(measurement_pack);
    }else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        UpdateLidar(measurement_pack);
    }
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.h
 */
void UKF::Prediction(double dt) {

    // Define augmented state vector
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    // Define Augmented Covariance matrix size
    P_aug_ = MatrixXd(7,7);
    P_aug_.fill(0.0);
    P_aug_.topLeftCorner(5,5) = P_;
    P_aug_(5,5) = std_a_ * std_a_;
    P_aug_(6,6) = std_yawdd_ * std_yawdd_;

    // Create square root matrix
    MatrixXd L = P_aug_.llt().matrixL();

    // Create augmented sigma points
    Xsig_aug.col(0) = x_aug;

    for(int i=0; i<n_aug_; i++){
        Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_)*L.col(i);
    }

    // predict sigma points
    for(int i=0; i<(2*n_aug_+ 1); i++){
        // extract values for better readability
        double px = Xsig_aug(0,i);
        double py = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);
        // predicted state values
        double px_p, py_p;
        // avoid division by zero
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
        // add noise
        px_p += 0.5*nu_a*dt*dt*cos(yaw);
        py_p += 0.5*nu_a*dt*dt*sin(yaw);
        vp += nu_a*dt;
        yaw_p += 0.5*dt*dt*nu_yawdd;
        yawd_p += nu_yawdd*dt;

        // write predicted sigma points
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = vp;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }

    // predict state mean and state covariance

    VectorXd x = VectorXd(n_x_);
    MatrixXd P = MatrixXd(n_x_,n_x_);
    x.fill(0.0);
    P.fill(0.0);

    // calculate predicted mean
    for(int i=0; i<(2*n_aug_+1); i++){
        x = x + weights_(i)*Xsig_pred_.col(i);
    }

    // calculate predicted covariance
    for(int i=0; i<(2*n_aug_ + 1); i++){
        VectorXd x_diff = Xsig_pred_.col(i) - x;
        x_diff(3) = constrainAngle(x_diff(3));

        P = P + weights_(i) * x_diff * x_diff.transpose();
    }

    // update prediction state
    x_ = x;
    P_ = P;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {

    // measurement prediction
    // Create matrix for sigma points in measurement space
    MatrixXd Zsig;
    // create Vector for predicted measeurement mean
    VectorXd z_pred;
    int n_z; // set measurement dimession Radar or Lidar
    // Measurement covariance matrix X
    MatrixXd S;
    // transform sigma points into radar measurement space
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
    MatrixXd z_diff = MatrixXd(n_z, 2*n_aug_+1);

    for(int i=0; i<(2*n_aug_+1); i++){
        //residual
        z_diff.col(i) = Zsig.col(i)-z_pred;
        S += weights_(i)*z_diff.col(i)*z_diff.col(i).transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_*std_laspx_, 0,
      0, std_laspy_*std_laspy_;

    S += R; // final S matrix

    // Create matrix for cross correlation
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    for(int i=0; i<(2*n_aug_+1); i++){
        //state difference
        VectorXd x_diff = Xsig_pred_.col(i)-x_;
        // angle normalization
        x_diff(3) = constrainAngle(x_diff(3));
        Tc += weights_(i)*x_diff*z_diff.col(i).transpose();
    }

    // Kalman gain K
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_residual = measurement_pack.raw_measurements_ - z_pred;

    // Calculate NIS
    NIS_laser_ = z_residual.transpose()*S.inverse()*z_residual;

    //update state mean and covariance matrix
    x_ = x_ + K * z_residual;
    P_ = P_ - K*S*K.transpose();

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {

    // measurement prediction
    // Create matrix for sigma points in measurement space
    MatrixXd Zsig;
    // create Vector for predicted measeurement mean
    VectorXd z_pred;
    int n_z; // set measurement dimession Radar or Lidar
    // Measurement covariance matrix X
    MatrixXd S;
    // transform predicted sigma points into radar measurement space
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
    MatrixXd z_diff = MatrixXd(n_z, 2*n_aug_+1);
    for(int i=0; i<(2*n_aug_+1); i++){
        //residual
        z_diff.col(i) = Zsig.col(i)-z_pred;
        S += weights_(i)*z_diff.col(i)*z_diff.col(i).transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_*std_radr_, 0, 0,
      0, std_radphi_*std_radphi_, 0,
      0, 0, std_radrd_*std_radrd_;

    S += R; // final S matrix

    // UKF Update
    // Create matrix for cross correlation
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    for(int i=0; i<(2*n_aug_+1); i++){
        //##z_diff.col(i)(1) = constrainAngle(z_diff.col(i)(1));
        //state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = constrainAngle(x_diff(3));
        Tc += weights_(i)*x_diff*z_diff.col(i).transpose();
    }

    // Kalman gain K
    MatrixXd K = Tc * S.inverse();
    //residual
    VectorXd z_residual = measurement_pack.raw_measurements_ - z_pred;
    //z_diff(1) = constrainAngle(z_diff(1));

    // Calculate NIS
    NIS_radar_ = z_residual.transpose()*S.inverse()*z_residual;

    //update state mean and covariance matrix
    x_ = x_ + K * z_residual;
    P_ = P_ - K*S*K.transpose();
}

/**
 * Constrain Angle between -pi and pi
 * @param angle
 * return constrained angle
 */

double UKF::constrainAngle(double ang){
    ang = fmod(ang + M_PI, 2.0*M_PI);
    if (ang < 0)
        ang += 2.0*M_PI;
    return ang - M_PI;
}


