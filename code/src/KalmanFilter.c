// inspo https://github.com/jarzebski/Arduino-KalmanFilter/blob/master/KalmanFilter.h
#include "KalmanFilter.h"

void initKalmanFilter (KalmanFilter* kf, double angle, double bias, double measure)
{
    kf->Q_angle = angle;
    kf->Q_bias = bias;
    kf->R_measure = measure;

    kf->K_angle = 0;
    kf->K_bias = 0;

    kf->P[0][0] = 1;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 1;
}

void update(KalmanFilter* kf,double newValue, double newRate, double dt)
{    //kf->dt = kf->ak - kf->kt;

    kf->K_rate = newRate - kf->K_bias;
    kf->K_angle += dt * kf->K_rate; // kalman_gain

    kf->P[0][0] += (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle) * dt;
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    kf->S = kf->P[0][0] + kf->R_measure;

    kf->K[0] = kf->P[0][0] / kf->S;
    kf->K[1] = kf->P[1][0] / kf->S;

    kf->y = newValue - kf->K_angle;

    kf->K_angle += kf->K[0] * kf->y;
    kf->K_bias += kf->K[1] * kf->y;

    kf->P[0][0] -= kf->K[0] * kf->P[0][0];
    kf->P[0][1] -= kf->K[0] * kf->P[0][1];
    kf->P[1][0] -= kf->K[1] * kf->P[0][0];
    kf->P[1][1] -= kf->K[1] * kf->P[0][1];

    //kf->kt = kf->ak;
};