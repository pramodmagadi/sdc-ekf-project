#include "kalman_filter.h"
#include "math.h"
#include<iostream>
using namespace std;
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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
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
}


VectorXd CartToPolar(VectorXd &x_state)
{
    float px, py, vx, vy;
    px = x_state(0);
    py = x_state(1);
    vx = x_state(2);
    vy = x_state(3);
    
    float rho = sqrt(px*px + py*py);
    float phi = atan2(py,px);
    
    if (rho < 0.00001)
    {
        cout << "Error converting radar cartesian coordinates to Polar - Division by 0" << endl;
        rho = 0.00001;
    }
    float rho_dot = (px*vx + py*vy)/rho;
    
    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    return z_pred;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  VectorXd z_pred = CartToPolar(x_);
  VectorXd y = z - z_pred;
  
  while(y(1) > M_PI)
  {
      y(1) -= 2*M_PI;
  }
  while(y(1) < -M_PI)
  {
      y(1) += 2*M_PI;
  }
  
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
}
