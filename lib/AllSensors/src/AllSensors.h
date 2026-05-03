#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_BMP3XX.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LSR_Struct.h>

#define SEALEVELPRESSURE_HPA (1013.25)

class AllSensors {
public:
    // Constructor: GPS UART, baud, LSM CS, BMP CS
    AllSensors(HardwareSerial *gpsSerialHardware, uint32_t gpsBaud = 9600, uint8_t lsmCS = 24, uint8_t bmpCS = 0);
    AllSensors(SoftwareSerial *gpsSerialSoftware, uint32_t gpsBaud = 9600, uint8_t lsmCS = 24, uint8_t bmpCS = 0);

    // Initialize all sensors
    bool begin();

    // Call repeatedly in loop
    void update();
    void updateNoKalmanFilter(LSR_Struct&);

    // Accelermeter Interrupt handlers
    void dataReadyLSMInt1(void);
    void dataReadyLSMInt2(void);
    const uint8_t getLSM6DSO32IntPin1(void);
    const uint8_t getLSM6DSO32IntPin2(void);

    // BMP Interrupt handler
    void dataReadyBMPInt(void);
    uint8_t getBMP390IntPin(void);

    // Manually set BMP sea-level pressure
    void calibrateBMPSeaLevel(void);
    void calibrateIMUGravityBias(void);

    // Get the altitude from the BMP
    const float getAltitudeBMP(void);
    const float getAltitudeBMP(float pressure_hPa);

    // Get the sea-level pressure from the BMP
    const float getSeaLevelPressure(void);

    volatile bool lsmDataReadyInt1; // gyro drdy
    volatile bool lsmDataReadyInt2; // accel drdy

private:
    // GPS
    HardwareSerial *gpsSerialHardware = nullptr;
    SoftwareSerial *gpsSerialSoftware = nullptr;
    uint32_t gpsBaud;
    TinyGPSPlus gps;

    // IMU
    Adafruit_LSM6DSO32 lsm;
    const uint8_t lsmCS;
    sensors_event_t accel, gyro, temp;
    const uint8_t lsmInterruptPin1 = 25, lsmInterruptPin2 = 26;
    float gyroBiasX = 0.2, gyroBiasY = 0.2, gyroBiasZ = 0.2;
    float accelBiasX = 0.2, accelBiasY = 0.2, accelBiasZ = 0.2;
    float velocityX = 0, velocityY = 0, velocityZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0;

    // BMP390
    Adafruit_BMP3XX bmp;
    const uint8_t bmpCS;
    volatile bool bmpDataReady;
    float bmpSeaLevel_hPa;

    // GPS
    double rocketLatitude = 0, rocketLongitude = 0, rocketSpeed = 0;
    double lastPrint = 0;
};

