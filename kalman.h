class KalmanFilter
{
public:
  KalmanFilter(float estimate, float initQ, float initR);
  float step(float measurement);
private:
  // initial values for the kalman filter
  float x_est_last;
  float P_last;

  // the noise in the system
  float Q;
  float R;

  float K;    // Kalman gain
  float P;
  float P_temp;
  float x_temp_est;
  float x_est;
};
