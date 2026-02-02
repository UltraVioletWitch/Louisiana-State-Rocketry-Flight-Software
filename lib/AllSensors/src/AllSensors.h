#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>
#include <LSR_Struct.h>

class AllSensors {
public:
    // Constructor: GPS UART, baud, LSM CS, BMP CS
    AllSensors(HardwareSerial &gpsSerial, uint32_t gpsBaud = 9600, int lsmCS = 10, int bmpCS = 9);

    // Initialize all sensors
    bool begin();

    // Call repeatedly in loop
    void update();

    // Manually set BMP sea-level pressure
    void calibrateBMPSeaLevel(void);

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

    double lastPrint = 0;
};

