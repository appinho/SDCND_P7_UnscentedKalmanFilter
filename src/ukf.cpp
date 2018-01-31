#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // Init bool to false
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  // TODO change to true
  use_radar_ = false;

  // Choose state dimensions
  n_x_ = 5;
  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd::Zero(n_x_);

  // initial covariance matrix
  P_ = MatrixXd::Zero(n_x_, n_x_);

  // init predicted sigma points matrix
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // Init weights
  weights_ = VectorXd::Zero(2 * n_aug_ + 1);

  // Choose lambda
  lambda_ = 3 - n_aug_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  // Initialize
  if(!is_initialized_){
    // Save time stamp
    time_us_ = meas_package.timestamp_;

    // Init state vector
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      // Convert Polar coordinates into Cartesian coordinates
      x_(0) = meas_package.raw_measurements_[0] 
        * cos(meas_package.raw_measurements_[1]);
      x_(1) = meas_package.raw_measurements_[0] 
        * sin(meas_package.raw_measurements_[1]);
    }
    else{
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }
    // TODO delete
    x_(2) = 10;
    x_(3) = -0.2;
    x_(4) = 0.1;

    // Init state covariance
    P_ <<  1,  0,  0,  0,  0,
           0,  1,  0,  0,  0,
           0,  0,  1,  0,  0,
           0,  0,  0,  1,  0,
           0,  0,  0,  0,  1;

    // Set init flag to true
    is_initialized_ = true;

    // Print status
    std::cout << "Initialization" << std::endl;
    std::cout << "x = " << std::endl << x_ << std::endl;
    std::cout << "P = " << std::endl << P_ << std::endl;
    std::cout << std::endl << std::endl;
  }
  // Process measurement
  else{
    // Calculate delta t and save current time stamp
    float delta_t = (meas_package.timestamp_ - time_us_)
       / 1000000.0;
    time_us_ = meas_package.timestamp_;

    // Prediction
    Prediction(delta_t);

    // Update
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
      UpdateRadar(meas_package);
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
      UpdateLidar(meas_package);
    }

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

  std::cout << "Predict with dt = " << delta_t << " s" << std::endl;


  // 1. Generate augmented sigma points
  //create augmented mean vector and state covariance
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++){
    Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  std::cout << "Generated sigma points " << std::endl << Xsig_aug << std::endl << std::endl;

  // 2. Predict sigma points
  for (int i = 0; i< 2 * n_aug_ + 1; i++){
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
    }
    else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  std::cout << "Predicted sigma points " << std::endl << Xsig_pred_ << std::endl << std::endl;

  // 3. Predict mean and covariance
  // Set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }
  std::cout << "Weights " << std::endl << weights_ << std::endl << std::endl;

  // Predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }

  // Print prediction
  std::cout << "Prediction" << std::endl;
  std::cout << "x = " << std::endl << x_ << std::endl;
  std::cout << "P = " << std::endl << P_ << std::endl;
  std::cout << std::endl << std::endl;

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
  std::cout << "Lidar Update with = " << std::endl 
    << meas_package.raw_measurements_ << std::endl;
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
  std::cout << "Radar Update with = " << std::endl 
    << meas_package.raw_measurements_ << std::endl;
}
