#include "kalman.h"

KalmanFilter::KalmanFilter(float estimate, float initQ, float initR)
{
  Q = initQ;
  R = initR;

  // initial values for the kalman filter
  x_est_last = 0;
  P_last = 0;

  // initialize with a measurement
  x_est_last = estimate;
}

// add a new measurement, return the Kalman-filtered value
float KalmanFilter::step(float z_measured)
{
  // do a prediction
  x_temp_est = x_est_last;
  P_temp = P_last + R*Q;

  // calculate the Kalman gain
  K = P_temp * (1.0/(P_temp + R));

  // correct
  x_est = x_temp_est + K * (z_measured - x_temp_est); 
  P = (1- K) * P_temp;

  // update our last's
  P_last = P;
  x_est_last = x_est;

  return (x_est);
}
