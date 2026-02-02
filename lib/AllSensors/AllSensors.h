#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>

class AllSensors {
public:
    // Constructor: GPS UART, baud, LSM CS, BMP CS
    AllSensors(HardwareSerial &gpsSerial, uint32_t gpsBaud = 9600, int lsmCS = 10, int bmpCS = 9);

    // Initialize all sensors
    bool begin();

    // Call repeatedly in loop
    void update();

    // Manually set BMP sea-level pressure
    void calibrateBMPSeaLevel(float seaLevel_hPa);

    // Start auto-calibration for BMP sea-level pressure
    void startBMPSeaLevelAutoCalibration(int readingsToAverage = 50);

    // Returns true if BMP auto-calibration is done
    bool isBMPAutoCalibrated();

private:
    // GPS
    HardwareSerial &gpsSerial;
    uint32_t gpsBaud;
    TinyGPSPlus gps;

    // IMU
    Adafruit_LSM6DSO32 lsm;
    int lsmCS;

    // BMP390
    Adafruit_BMP3XX bmp;
    int bmpCS;
    float bmpSeaLevel_hPa;

    // Auto-calibration state
    bool bmpAutoCalibrating;
    int bmpDiscardCount;
    int bmpReadingsCount;
    float bmpPressureSum;
    int bmpCalibrationReadings;

    unsigned long lastPrint;
};

