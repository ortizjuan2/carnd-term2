#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    double rho = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
    double phi = 0; //atan2(x_[1], x_[0]);
    double rho_dot;

    if(x_[0] != 0 || x_[1] != 0)
    	phi = atan2(x_[1], x_[0]);

    if(rho != 0.0){
        rho_dot = (x_[0]*x_[2] + x_[1]*x_[3])/rho;
    }else{
        rho_dot = 0;
    }

    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    VectorXd y = z - z_pred;
    MatrixXd Ht = Hj_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = Hj_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj_) * P_;

}
