#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define EPS 0.0001

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    is_initialized_ = false;
    use_laser_ = true;
    use_radar_ = true;
    x_ = VectorXd(5);
    P_ = MatrixXd(5, 5);
    std_a_ = 30;
    std_yawdd_ = 30;

    // Start don't tune
    std_laspx_ = 0.15;
    std_laspy_ = 0.15;
    std_radr_ = 0.3;
    std_radphi_ = 0.03;
    std_radrd_ = 0.3;
    // End don't tune

    // Unscented transform initialization
    n_x_ = x_.size();
    n_aug_ = n_x_ + 2;
    n_sig_ = 2*n_aug_+1;
    lambda_ = 3 - n_aug_;

    Xsig_pred_ = MatrixXd(n_x_, n_sig_);
    weights_ = VectorXd(n_sig_);

    // Measurement noise initialization
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << std_radr_*std_radr_, 0, 0,
                0, std_radphi_*std_radphi_, 0,
                0, 0,std_radrd_*std_radrd_;
    R_lidar_ = MatrixXd(2, 2);
    R_lidar_ << std_laspx_*std_laspx_,0,
                0,std_laspy_*std_laspy_;
}

UKF::~UKF() {}
double UKF::NormalizeAngles(double theta){
    while(theta > M_PI) theta -= 2.*M_PI;
    while(theta < -M_PI) theta += 2.*M_PI;

    return theta;
}
/**
 * @param {MeasurementPackage} measPkg The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measPkg) {
    if (!is_initialized_) {
        // Initial covariance matrix
        P_ << 1, 0, 0, 0, 0,
           0, 1, 0, 0, 0,
           0, 0, 1, 0, 0,
           0, 0, 0, 1, 0,
           0, 0, 0, 0, 1;
        if (measPkg.sensor_type_ == MeasurementPackage::RADAR) {
            // Convert radar from polar to cartesian coordinates and initialize state.
            float rho = measPkg.raw_measurements_[0]; // range
            float phi = measPkg.raw_measurements_[1]; // bearing
            float rho_dot = measPkg.raw_measurements_[2]; // velocity of rho
            // Coordinates convertion from polar to cartesian
            float px = rho * cos(phi);
            float py = rho * sin(phi);
            float vx = rho_dot * cos(phi);
            float vy = rho_dot * sin(phi);
            float v  = sqrt(vx * vx + vy * vy);
            x_ << px, py, v, 0, 0;
        }
        else if (measPkg.sensor_type_ == MeasurementPackage::LASER) {
            // We don't know velocities from the first measurement of the LIDAR, so, we use zeros
            x_ << measPkg.raw_measurements_[0], measPkg.raw_measurements_[1], 0, 0, 0;
            // Deal with the special case initialisation problems
            if (fabs(x_(0)) < EPS and fabs(x_(1)) < EPS){
                x_(0) = EPS;
                x_(1) = EPS;
            }
        }

        // Initialize weights
        weights_(0) = lambda_ / (lambda_ + n_aug_);
        for (int i = 1; i < weights_.size(); i++) {
            weights_(i) = 0.5 / (n_aug_ + lambda_);
        }

        // Save timestamp for dt calculation
        time_us_ = measPkg.timestamp_;
        is_initialized_ = true;

        return;
    }

    // Calculate the timestep between measurements in seconds
    double dt = (measPkg.timestamp_ - time_us_);
    dt /= 1000000.0; // convert micros to s
    time_us_ = measPkg.timestamp_;

    // Predict
    Prediction(dt);

    if (measPkg.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        UpdateRadar(measPkg);
    }
    else if (measPkg.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        UpdateLidar(measPkg);
    }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} dt the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {

    // Augmented state matrix
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.fill(0.0);
    x_aug.head(n_x_) = x_;

    // Augmented process covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0);
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;
    // Compute the square root of the covariance matrix
    MatrixXd L = P_aug.llt().matrixL();

    // Compute the sigma points
    MatrixXd Xsig = MatrixXd(n_aug_, n_sig_);
    Xsig.col(0) = x_aug;
    for(int i = 0; i < n_aug_; i++) {
        Xsig.col(i+1)        = x_aug + sqrt(lambda_+n_aug_)*L.col(i);
        Xsig.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_)*L.col(i);
    }

    // Predict sigma points
    for (int i = 0; i < n_sig_; i++){
        // Extract values for better readability
        double p_x  = Xsig(0,i);
        double p_y  = Xsig(1,i);
        double v    = Xsig(2,i);
        double yaw  = Xsig(3,i);
        double yawd = Xsig(4,i);
        double nu_a = Xsig(5,i);
        double nu_yawdd = Xsig(6,i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
        }
        else {
            px_p = p_x + v*dt*cos(yaw);
            py_p = p_y + v*dt*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*dt;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*dt*dt * cos(yaw);
        py_p = py_p + 0.5*nu_a*dt*dt * sin(yaw);
        v_p = v_p + nu_a*dt;

        yaw_p = yaw_p + 0.5*nu_yawdd*dt*dt;
        yawd_p = yawd_p + nu_yawdd*dt;

        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }

    // Compute predicted state mean and covariance
    x_ = Xsig_pred_ * weights_; // vectorised sum
    P_.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points
        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // Angle normalization
        x_diff(3) = NormalizeAngles(x_diff(3));
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} measPkg
 */
void UKF::UpdateLidar(MeasurementPackage measPkg) {
    int n_z = 2;

    // Transform state predictions to measurement space. Since the points are
    // already in the state matrix, this only means choosing the relevant
    // states from the prediction matrix
    MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);
    UpdateCore_(measPkg, Zsig, n_z, R_lidar_, &NIS_lidar_);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} measPkg
 */
void UKF::UpdateRadar(MeasurementPackage measPkg) {
    int n_z = 3;
    MatrixXd Zsig = MatrixXd(n_z, n_sig_);
    // Transform the state predictions to measurement space

    for (int i = 0; i < n_sig_; i++) {
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;
        // Measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);          //r
        Zsig(1,i) = atan2(p_y,p_x);                   //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / Zsig(0,i);   //r_dot
    }

    UpdateCore_(measPkg, Zsig, n_z, R_radar_, &NIS_radar_);
}


void UKF::UpdateCore_(MeasurementPackage measPkg, MatrixXd Zsig, int n_z, MatrixXd R, double* NIS){

    // Mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred  = Zsig * weights_;
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {
        // Residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // Angle normalization
        z_diff(1) = NormalizeAngles(z_diff(1));
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    S = S + R;

    // Create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    // Calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        if (measPkg.sensor_type_ == MeasurementPackage::RADAR){ // Radar
            // Angle normalization
            z_diff(1) = NormalizeAngles(z_diff(1));
        }
        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // Angle normalization
        x_diff(3) = NormalizeAngles(x_diff(3));
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    // Measurements
    VectorXd z = measPkg.raw_measurements_;
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    // Residual
    VectorXd z_diff = z - z_pred;
    if (measPkg.sensor_type_ == MeasurementPackage::RADAR){ // Radar
        // Angle normalization
        z_diff(1) = NormalizeAngles(z_diff(1));
    }
    // Update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // Calculate NIS
    *NIS = z.transpose() * S.inverse() * z;
}

