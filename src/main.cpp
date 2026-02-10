#include <Arduino.h>
#include "EKF.h"
#include "AllSensors.h"

// --------------------
// Teensy pins / serial
// --------------------
#define LSM_CS 10
#define BMP_CS 9
#define GPS_SERIAL Serial1
#define GPS_BAUD 9600

// --------------------
// Objects
// --------------------
AllSensors sensors(GPS_SERIAL, GPS_BAUD, LSM_CS, BMP_CS);
EKF ekf(0.01f);  // 100 Hz

// Time tracking
unsigned long lastLoop = 0;
const float dt = 0.01f; // 100 Hz

// EKF state initialization
float x0[16] = {1,0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0};
float P0[16][16];

// GPS reference
static bool gpsRefSet = false;
static float refLat = 0.0f, refLon = 0.0f;

// Horizontal acceleration lock
static bool gpsLocked = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("=== Starting EKF with Sensors ==="));

    // Initialize sensors
    if (!sensors.begin()) {
        Serial.println(F("Sensor initialization failed!"));
        while (1) delay(100);
    }

    // Initialize EKF state
    for(int i=0;i<16;i++){
        for(int j=0;j<16;j++) P0[i][j]=0.0f;
        P0[i][i]=1.0f;
    }
    ekf.init(x0,P0);
}

void loop() {
    unsigned long now = millis();
    if (now - lastLoop < 10) return; // 100 Hz
    lastLoop = now;

    // --------------------
    // Read sensors individually
    // --------------------
    sensors.readIMU();
    sensors.readGPS();
    sensors.readBMP();

    // --------------------
    // Prepare IMU data
    // --------------------
    float gyro[3]  = { sensors.gyroX, sensors.gyroY, sensors.gyroZ };
    float accel[3] = { sensors.AccelX, sensors.AccelY, sensors.AccelZ };

    // --------------------
    // GPS lock check
    // --------------------
    if (sensors.isValid) {
        gpsLocked = true; // we have a valid fix
        if (!gpsRefSet) {
            refLat = sensors.lat;
            refLon = sensors.lon;
            gpsRefSet = true;
        }
    }

    // --------------------
    // Apply horizontal lock if GPS not available
    // --------------------
    if (!gpsLocked) {
        accel[0] = 0.0f; // x acceleration
        accel[1] = 0.0f; // y acceleration
        // vertical acceleration still integrated
    }

    // --------------------
    // EKF predict
    // --------------------
    ekf.predict(gyro, accel);

    // --------------------
    // GPS update (only if valid)
    // --------------------
    if (sensors.isValid) {
        float dx = (sensors.lon - refLon) * 111320.0 * cos(refLat*3.14159/180.0);
        float dy = (sensors.lat - refLat) * 110540.0;
        float dz = sensors.alt;

        float gpsXYZ[3] = { dx, dy, dz };
        ekf.updateGPS(gpsXYZ);
    }

    // --------------------
    // Pressure update
    // --------------------
    float bmpPressure_hPa = sensors.pressure / 100.0;
    float altitude_m = 44330.0 * (1.0 - pow(bmpPressure_hPa / sensors.bmpSeaLevel_hPa, 0.1903));
    ekf.updatePressure(altitude_m);

    // --------------------
    // EKF outputs
    // --------------------
    float q[4];  // quaternion
    float p[3];  // position
    ekf.getQuaternion(q);
    ekf.getPosition(p);

    // --------------------
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    // --------------------
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    float pitch = asinf(2.0f*(q0*q2 - q3*q1));
    float yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));

    // Convert radians to degrees
    roll  *= 180.0f / PI;
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;

    // --------------------
    // Print Euler angles and position
    // --------------------
    Serial.print("Roll: "); Serial.print(roll,2);
    Serial.print(" Pitch: "); Serial.print(pitch,2);
    Serial.print(" Yaw: "); Serial.print(yaw,2);
    Serial.print(" | Pos [m]: ");
    for(int i=0;i<3;i++) { Serial.print(p[i],2); Serial.print(" "); }
    Serial.println();
}

