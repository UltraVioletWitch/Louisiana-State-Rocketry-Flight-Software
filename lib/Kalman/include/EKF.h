#pragma once
#include <Arduino.h>
#include <math.h>

class EKF {
public:
    EKF(float dt_);
    void init(const float x0[16], const float P0[16][16]);

    void predict(const float gyro[3], const float accel[3]);
    void updateGPS(const float gps[3]);
    void updatePressure(float altitude);

    void getQuaternion(float q_out[4]) const;
    void getPosition(float p_out[3]) const;

private:
    float dt;
    float x[16];       // state vector
    float P[16][16];   // covariance
    float Q[16][16];   // process noise
    float R_gps[3][3]; // GPS measurement noise
    float R_press;     // pressure measurement noise

    void normalizeQuat(float q[4]);
    void quatToRotMat(const float q[4], float R[3][3]);
};

