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

  is_initialized = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar = true;

  // initial state vector
  x_ = VectorXd(5);

  x_ << 1, 1, 1, 1, 0.1;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);


  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd = 0.5;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...

  */

  // State dimension
  n_x= 5;

  // Augmented state dimension
  n_aug= n_x + 2;


  // Sigma points dimension
  n_sig = 2 * n_aug + 1;

  //Sigma point spreading parameter for augmented sigma points
  lambda = 3 - n_aug;

  // Weights of sigma points
  weights_ = VectorXd(n_sig);
  weights_.fill (0.5/(lambda + n_aug));
  weights_(0) = lambda / (lambda + n_aug);

  // Matrix to hold sigma points
  Xsig_pred_ = MatrixXd(n_x, n_sig);

  // Measurement noise covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr*std_radr, 0, 0,
              0, std_radphi*std_radphi, 0,
              0, 0,std_radrd*std_radrd;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx*std_laspx,0,
              0,std_laspy*std_laspy;

			  
  H_laser_ = MatrixXd(2, 5);
  H_laser_  << 1,0,0,0,0,
               0,1,0,0,0;
			  
			  
  previous_timestamp = 0;

  NIS_laser = 0;
  NIS_radar = 0; 
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  // Initialisation

  if (!is_initialized) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */


    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      double vx = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(2));
      double vy = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(2));
      double v = sqrt(vx*vx + vy*vy);

      x_(0) = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
      x_(1) = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
      x_(2) = v;
      x_(3) = meas_package.raw_measurements_(1) ;
      x_(4) = 0;

	  P_ << std_radr*std_radr, 0, 0, 0, 0,
            0, std_radr*std_radr, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, std_radphi, 0,
            0, 0, 0, 0, std_radphi;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      x_(2) = 0;
      x_(3) = 0;
      x_(4) = 0;
	  P_ << std_laspx*std_laspx, 0, 0, 0, 0,
            0, std_laspy*std_laspy, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    }


    previous_timestamp = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized = true;
    return;
  }

  // Prediction
  float dt = (meas_package.timestamp_ - previous_timestamp)/ 1000000.0 ;

  previous_timestamp = meas_package.timestamp_;

  Prediction(dt);

  // UpdateRadar

 if (use_radar == true) {
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

        UpdateRadar(meas_package);
  }
 }

  // UpdateLidar
if (use_laser == true) {
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

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


  // Augmented state vector

  VectorXd x_aug_ = VectorXd(n_aug);
  x_aug_.head(5) = x_;
  x_aug_(5)      = 0;
  x_aug_(6)      = 0;


  // Augmented state covariance

  MatrixXd P_aug_ = MatrixXd(n_aug, n_aug);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x, n_x) = P_;
  P_aug_(5,5) = std_a*std_a; // Q - Matrix
  P_aug_(6,6) = std_yawdd*std_yawdd; // Q - Matrix

  // square root matrix

  MatrixXd L_ = P_aug_.llt().matrixL();

  // Generate Sigma Points

  MatrixXd Xsig_aug_ = MatrixXd(n_aug, n_sig);

  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i< n_aug; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug_ + std::sqrt(lambda+n_aug) * L_.col(i);
    Xsig_aug_.col(i+1+n_aug) = x_aug_ - std::sqrt(lambda+n_aug) * L_.col(i);
  }

  // Predict Sigma Points

  for (int i = 0; i< 2*n_aug+1; i++)
  {
    //extract values for better readability
    double p_x      = Xsig_aug_(0,i);
    double p_y      = Xsig_aug_(1,i);
    double v        = Xsig_aug_(2,i);
    double yaw      = Xsig_aug_(3,i);
    double yawd     = Xsig_aug_(4,i);
    double nu_a     = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p; // predicated position
     //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }


    double v_p = v; //predicated velocity - constant
    double yaw_p = yaw + yawd*delta_t; // predicated yaw
    double yawd_p = yawd; // predicated yaw rate - constant

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

    // predicted mean

    x_.fill(0.0);
    for (int i = 0; i < n_sig; i++) {
        x_ = x_ + weights_(i)* Xsig_pred_.col(i);


    }
    // predicated state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < n_sig; i++) {  //iterate over sigma points

    // state difference
        VectorXd x_diff_ = Xsig_pred_.col(i) - x_;
    //angle normalization
        if (x_diff_(3)> M_PI) x_diff_(3)-=2.*M_PI;
        if (x_diff_(3)<-M_PI) x_diff_(3)+=2.*M_PI;

        P_ = P_ + weights_(i) * x_diff_ * x_diff_.transpose() ;
  }


}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  
  
  TODO:
	* update the state by using Kalman Filter equations
	
	
	*/
	
	VectorXd z_meas = VectorXd(2);
	
	z_meas(0) = meas_package.raw_measurements_(0);
    z_meas(1) = meas_package.raw_measurements_(1);
  
	
	VectorXd y = z_meas - (H_laser_ * x_);
	MatrixXd H_trans = H_laser_.transpose();
	MatrixXd S = H_laser_ * P_ * H_trans + R_laser_;
	MatrixXd S_inv = S.inverse();
	MatrixXd K = P_ * H_trans * S_inv;

	// Updated State and State Covariance
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	x_ = x_ + (K * y);
	P_ = (I - K * H_laser_) * P_;
 
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

  // PREDICT MEASUREMENT

  // measurement vector dimension

  int n_z = 3;

  // sigma points in measurement space

  MatrixXd Zsig_ = MatrixXd(n_z, n_sig);
  Zsig_.fill(0.0);

 // sigma points in measurement space

  for (int i = 0; i < n_sig; i++) {

    double p_x      = Xsig_pred_(0,i);
    double p_y      = Xsig_pred_(1,i);
    double v        = Xsig_pred_(2,i);
    double yaw      = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    Zsig_(0,i) = std::sqrt((p_x*p_x) +  (p_y*p_y));
    Zsig_(1,i) = std::atan2(p_y, p_x);
    Zsig_(2,i) = (p_x*v1 + p_y*v2 ) / std::sqrt(p_x*p_x + p_y*p_y);

  }

  //mean predicted measurement

   VectorXd z_pred_ = VectorXd(n_z);
   z_pred_.fill(0.0);
   for (int i=0; i < n_sig; i++) {
       z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
  }
  // measurement covariance matrix

  MatrixXd S_ = MatrixXd(n_z, n_z);
  S_.fill(0.0);

  for (int i = 0; i < n_sig; i++) {  //2n+1 simga points

       VectorXd z_diff_ = Zsig_.col(i) - z_pred_; //residual

       if (z_diff_(1)> M_PI) z_diff_(1)-=2.*M_PI;
       if (z_diff_(1)<-M_PI) z_diff_(1)+=2.*M_PI;

       S_ = S_ + weights_(i) * z_diff_ * z_diff_.transpose();

  }

  // adding measurement noise cov. matrix

  S_ = S_ + R_radar_;


// UPDATE STATE UND COVARIANCE


  VectorXd z_ = meas_package.raw_measurements_;

  // cross correlation matrix

  MatrixXd Tc_ = MatrixXd(n_x, n_z);
  Tc_.fill(0.0);

  for (int i = 0; i < n_sig; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff_ = Zsig_.col(i) - z_pred_;
	if (z_diff_(1)> M_PI) z_diff_(1)-=2.*M_PI;
    if (z_diff_(1)<-M_PI) z_diff_(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff_ = Xsig_pred_.col(i) - x_;
    if (x_diff_(3)> M_PI) x_diff_(3)-=2.*M_PI;
    if (x_diff_(3)<-M_PI) x_diff_(3)+=2.*M_PI;

    Tc_ = Tc_ + weights_(i) * x_diff_ * z_diff_.transpose();
   }

   //Kalman gain K;
  MatrixXd K_ = Tc_ * S_.inverse();

  //residual
  VectorXd z_diff_ = z_ - z_pred_;

  if (z_diff_(1)> M_PI) z_diff_(1)-=2.*M_PI;
  if (z_diff_(1)<-M_PI) z_diff_(1)+=2.*M_PI;
  
  
  //update state mean and covariance matrix
  x_ = x_ + K_ * z_diff_;
  P_ = P_ - K_*S_*K_.transpose();

  //NIS Radar Update
  NIS_radar = z_diff_.transpose() * S_.inverse() * z_diff_;


}
