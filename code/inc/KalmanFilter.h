#ifndef KALMAN_FILTER_HH
#define KALMAN_FILTER_HH

typedef struct KalmanFilter{
    double Q_angle, Q_bias, R_measure;
	double K_angle, K_bias, K_rate;
	double P[2][2], K[2];
	double S, y;
} KalmanFilter;

void initKalmanFilter (KalmanFilter* kalmanfilter, double angle, double bias, double measure);
void update(KalmanFilter* kalmanfilter, double newValue, double newRate, double dt);

#endif