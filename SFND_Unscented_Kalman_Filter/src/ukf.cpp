#include "ukf.h"
#include "Eigen/Dense"
#include<fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 4;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
    is_initialized_ = false;

    // State dimension
    n_x_ = 5;

    // Augmented state dimension
    n_aug_ = 7;

    // Sigma point spreading parameter
    lambda_ = 3 - n_x_;

    // predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    Xsig_pred_.fill(0.0);

    // Weights of sigma points
    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_.fill(0.5 / (lambda_ + n_aug_));
    weights_(0) = lambda_ / (lambda_ + n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    /**
     * Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */
    if (!is_initialized_)
    {
        if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            // initialize using lidar
            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
        } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            // initialize using radar
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            double rho_dot = meas_package.raw_measurements_[2];

            x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
        }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
    }
    double delta = (meas_package.timestamp_ - time_us_) / 1000000.0L;
    time_us_ = meas_package.timestamp_;
    //Prediction of next position
    Prediction(delta);

    //checking sensor type and then calling the respective update fun
    if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    {
        UpdateLidar(meas_package);
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {
        UpdateRadar(meas_package);
    }

}



void UKF::Prediction(double delta_t) {
  /**
   * Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

    // creating augmented state mean vector and state covariance matrix
    VectorXd x_aug = VectorXd::Zero(n_aug_);
    x_aug.head(5) = x_;
    x_aug(5) = 0; // velocity acceleration
    x_aug(6) = 0; // angle acceleration

    MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5)=  P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    // creating augmented sigma point matrix
    MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);

    MatrixXd L = P_aug.llt().matrixL();
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; ++i)
    {
        Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }


    // predicting sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        // extract state
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        // predicted state values
        double px_p, py_p, v_p, yaw_p, yawd_p;

        // avoid division by zero
        if (fabs(yawd) > 0.001)
        {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }

        v_p = v;
        yaw_p = yaw + yawd*delta_t;
        yawd_p = yawd;

        // adding noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        // update Xsig_pred
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }


    // predicting state mean and covariance
    VectorXd final_x = VectorXd::Zero(5);
    MatrixXd final_P = MatrixXd::Zero(5, 5);

    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        final_x += weights_(i) * Xsig_pred_.col(i);
    }

    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        VectorXd x_diff = Xsig_pred_.col(i) - final_x;

        while (x_diff(3) > M_PI) x_diff(3) -= 2.0*M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

        final_P += weights_(i) * x_diff * x_diff.transpose();
    }


    // write result
    x_ = final_x;
    P_ = final_P;

}


void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
    int n_z = 2;
    MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);
    Eigen::VectorXd z_pred = VectorXd::Zero(n_z);
    Eigen::MatrixXd S = MatrixXd::Zero(n_z, n_z);

    // predicting the measurements

    // transforming sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        // extract value
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        double yawd = Xsig_pred_(4, i);

        // measurement model
        Zsig(0, i) = p_x;   // p_x
        Zsig(1, i) = p_y;   // p_y
    }


    // calculate predict mean and covariance in measurement space

    // measurement mean
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        z_pred += weights_(i) * Zsig.col(i);
    }

    // measurement covariance
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;

        while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

        S += weights_(i) * z_diff * z_diff.transpose();
    }

    // adding measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;
    S = S + R;


    // update state mean and covariance
    VectorXd z = meas_package.raw_measurements_; // measurement received from sensor
    MatrixXd Tc = MatrixXd::Zero(n_x_, n_z); // cross correlation matrix tc

    // calculate Tc
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        while (x_diff(3) > M_PI) x_diff(3) -= 2.0*M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

        // measurement difference
        VectorXd z_diff = Zsig.col(i) - z_pred;

        while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }


    // calculating kalman gain K
    MatrixXd K = Tc * S.inverse();

    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    // update state mean and covariance
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // calculating NIS
    std::ofstream log("../NIS_Laser.txt",std::ios_base::app | std::ios_base::out);
    log<< (z-z_pred).transpose() * S.inverse() * (z-z_pred);
    log << "\n";


}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

    int n_z = 3;
    MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);
    Eigen::VectorXd z_pred = VectorXd::Zero(n_z);
    Eigen::MatrixXd S = MatrixXd::Zero(n_z, n_z);

    // predicting the measurements and transforming sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        // extract value
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        double yawd = Xsig_pred_(4, i);

        // measurement model
        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);    // r
        Zsig(1, i) = atan2(p_y, p_x);                // phi
        Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);    // r_dot
    }

    // calculate predict mean and covariance in measurement space

    // measurement mean
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        z_pred += weights_(i) * Zsig.col(i);
    }

    // measurement covariance mat
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;

        while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

        S += weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0, std_radrd_*std_radrd_;
    S = S + R;


    // updating state mean and covariance
    VectorXd z = meas_package.raw_measurements_; // received measurement
    MatrixXd Tc = MatrixXd::Zero(n_x_, n_z); // cross correlation matrix

    // calculate Tc
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        while (x_diff(3) > M_PI) x_diff(3) -= 2.0*M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

        // measurement difference
        VectorXd z_diff = Zsig.col(i) - z_pred;

        while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }


    // calculate kalman gain K
    MatrixXd K = Tc * S.inverse();

    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    // update state mean and covariance
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    //calculating NIS for RADAR
    std::ofstream log_R("../NIS_Radar.txt",std::ios_base::app | std::ios_base::out);
    log_R<< (z-z_pred).transpose() * S.inverse() * (z-z_pred);
    log_R << "\n";
}