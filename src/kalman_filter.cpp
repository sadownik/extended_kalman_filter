#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}



void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_lin, MatrixXd &R_rin, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  //R_ = R_in;
  R_laser = R_lin;
  R_radar = R_rin;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  /*
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  */
  
  
  VectorXd y;  
  y = z - H_ * x_;
 

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  MatrixXd I = MatrixXd::Identity(4, 4);
  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
  
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  VectorXd y;
  MatrixXd Hj = tools.CalculateJacobian(x_);
  VectorXd x_polar = tools.ConvertFromCartesianToPolarCoords(x_);
  
    y = z - x_polar;

    // normalize the angle between -pi to pi
    while(y(1) > M_PI){
      y(1) -= 2 * M_PI;
    }

    while(y(1) < -M_PI){
      y(1) += 2 * M_PI;
    }
  cout << "test pin radar"<< endl;
  MatrixXd Ht = Hj.transpose();
  cout << Hj << "H_" << endl << P_ << "P_" << Ht << endl;
  MatrixXd S = Hj * P_ * Ht + R_radar;
  cout << "test pin radar end"<< endl;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  MatrixXd I = MatrixXd::Identity(4, 4);
  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * Hj) * P_;
  
  
  
}
