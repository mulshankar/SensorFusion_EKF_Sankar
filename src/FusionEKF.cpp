#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

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
  
	H_laser_<< 1,0,0,0,
			0,1,0,0;
	
	noise_ax = 9.0;
	noise_ay = 9.0;
	
	//process covariance matrix
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;
	
	// state noise matrix
	ekf_.Q_ = MatrixXd(4, 4);

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);

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

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      ekf_.x_(0)=measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]);
	  ekf_.x_(1)=measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]);
	  ekf_.x_(2)=0;
	  ekf_.x_(3)=0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_(0)=measurement_pack.raw_measurements_[0];
	  ekf_.x_(1)=measurement_pack.raw_measurements_[1];
	  ekf_.x_(2)=0;
	  ekf_.x_(3)=0;
    }
	
	ekf_.F_ << 1, 0, 1, 0,
		  0, 1, 0, 1,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
	
	float previous_timestamp=measurement_pack.timestamp_;
	
	ekf_.Q_ << 1, 0, 1, 0,
		  0, 1, 0, 1,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
	
	// done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
	
    // TODO: YOUR CODE HERE
	//1. Modify the F matrix so that the time is integrated
	
	ekf_.F_ << 1, 0, dt, 0,
			  0, 1, 0, dt,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
	
	//2. Set the process covariance matrix Q
	
	ekf_.Q_ << (pow(dt,4)*noise_ax)/4, 0, (pow(dt,3)*noise_ax)/2, 0,
			  0, (pow(dt,4)*noise_ay)/4, 0, (pow(dt,3)*noise_ay)/2,
			  (pow(dt,3)*noise_ax)/2, 0, (pow(dt,2)*noise_ax), 0,
			  0, (pow(dt,3)*noise_ay)/2, 0, (pow(dt,2)*noise_ay);
   
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    MatrixXd Hj_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.H_=Hj_;
	ekf_.R_=R_radar_;
  } 
  else{
  ekf_.H_=H_laser_;
  ekf_.R_=R_laser_;    
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
