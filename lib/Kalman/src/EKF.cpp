#include "EKF.h"
#include <math.h>

//---------------------------------------------
// Constructor
EKF::EKF(float dt_) : dt(dt_) {
    for(int i=0;i<16;i++){
        x[i]=0.0f;
        for(int j=0;j<16;j++) P[i][j]=0.0f;
    }
    x[0]=1.0f; // quaternion w=1

    for(int i=0;i<16;i++) P[i][i]=1.0f;

    // Process noise
    for(int i=0;i<16;i++){
        for(int j=0;j<16;j++) Q[i][j]=0.0f;
        Q[i][i]=1e-5f;
    }

    // GPS noise
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++) R_gps[i][j]=0.0f;
        R_gps[i][i]=3.0f;
    }

    // Pressure noise
    R_press=1.0f;
}

//---------------------------------------------
void EKF::init(const float x0[16], const float P0[16][16]){
    for(int i=0;i<16;i++) x[i]=x0[i];
    for(int i=0;i<16;i++) for(int j=0;j<16;j++) P[i][j]=P0[i][j];
}

//---------------------------------------------
// Normalize quaternion
void EKF::normalizeQuat(float q[4]){
    float norm = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    for(int i=0;i<4;i++) q[i]/=norm;
}

//---------------------------------------------
// Quaternion to rotation matrix
void EKF::quatToRotMat(const float q[4], float R[3][3]){
    float qw=q[0], qx=q[1], qy=q[2], qz=q[3];
    R[0][0] = 1-2*qy*qy-2*qz*qz; R[0][1] = 2*qx*qy-2*qz*qw; R[0][2] = 2*qx*qz+2*qy*qw;
    R[1][0] = 2*qx*qy+2*qz*qw;   R[1][1] = 1-2*qx*qx-2*qz*qz; R[1][2] = 2*qy*qz-2*qx*qw;
    R[2][0] = 2*qx*qz-2*qy*qw;   R[2][1] = 2*qy*qz+2*qx*qw;   R[2][2] = 1-2*qx*qx-2*qy*qy;
}

//---------------------------------------------
// Prediction (IMU)
void EKF::predict(const float gyro[3], const float accel[3]){
    float q[4]; for(int i=0;i<4;i++) q[i]=x[i];
    float v[3]; for(int i=0;i<3;i++) v[i]=x[4+i];
    float p[3]; for(int i=0;i<3;i++) p[i]=x[7+i];
    float bg[3]; for(int i=0;i<3;i++) bg[i]=x[10+i];
    float ba[3]; for(int i=0;i<3;i++) ba[i]=x[13+i];

    // Bias-corrected gyro and accel
    float omega[3], a[3];
    for(int i=0;i<3;i++){
        omega[i]=gyro[i]-bg[i];
        a[i]=accel[i]-ba[i];
    }

    // Quaternion update (Euler integration)
    float q_dot[4];
    q_dot[0] = -0.5f*(omega[0]*q[1] + omega[1]*q[2] + omega[2]*q[3]);
    q_dot[1] =  0.5f*(omega[0]*q[0] + omega[1]*q[3] - omega[2]*q[2]);
    q_dot[2] =  0.5f*(omega[1]*q[0] - omega[0]*q[3] + omega[2]*q[1]);
    q_dot[3] =  0.5f*(omega[2]*q[0] + omega[0]*q[2] - omega[1]*q[1]);
    for(int i=0;i<4;i++) q[i] += q_dot[i]*dt;
    normalizeQuat(q);

    // Velocity update: convert accel to world frame and remove gravity
    float R[3][3]; quatToRotMat(q,R);
    float g[3]={0.0f,0.0f,-9.81f};
    float a_world[3];
    for(int i=0;i<3;i++){
        a_world[i] = a[0]*R[i][0] + a[1]*R[i][1] + a[2]*R[i][2] - g[i];
        v[i] += a_world[i]*dt;
    }

    // Position update
    for(int i=0;i<3;i++) p[i] += v[i]*dt;

    // Update state vector
    for(int i=0;i<4;i++) x[i]=q[i];
    for(int i=0;i<3;i++) x[4+i]=v[i];
    for(int i=0;i<3;i++) x[7+i]=p[i];

    // Simple covariance propagation (diagonal only)
    for(int i=0;i<16;i++) P[i][i] += Q[i][i];
}

//---------------------------------------------
// GPS update
void EKF::updateGPS(const float gps[3]){
    float y[3];
    for(int i=0;i<3;i++) y[i]=gps[i]-x[7+i];

    // Kalman gain K (16x3), approximate diagonal-only
    float K[16][3];
    for(int i=0;i<16;i++){
        for(int j=0;j<3;j++){
            K[i][j] = (i>=7 && i<10 && i==7+j) ? P[i][i]/(P[i][i]+R_gps[j][j]) : 0.0f;
        }
    }

    // Update state
    for(int i=0;i<16;i++){
        for(int j=0;j<3;j++) x[i] += K[i][j]*y[j];
    }

    // Update covariance (approx diagonal)
    for(int i=7;i<10;i++){
        P[i][i] *= (1-K[i][i-7]);
    }
}

//---------------------------------------------
// Pressure update
void EKF::updatePressure(float altitude){
    float y = altitude - x[9];

    float K[16];
    float S = P[9][9] + R_press;
    for(int i=0;i<16;i++) K[i] = P[i][9]/S;

    // Update state
    for(int i=0;i<16;i++) x[i] += K[i]*y;

    // Update covariance
    for(int i=0;i<16;i++) P[i][9] *= (1-K[i]);
}

//---------------------------------------------
// Get quaternion
void EKF::getQuaternion(float q_out[4]) const{
    for(int i=0;i<4;i++) q_out[i]=x[i];
}

//---------------------------------------------
// Get position
void EKF::getPosition(float p_out[3]) const{
    for(int i=0;i<3;i++) p_out[i]=x[7+i];
}

