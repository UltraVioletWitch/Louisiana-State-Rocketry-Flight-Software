#pragma once

#include <Arduino.h>
#include <SPI.h>

// IMU (LSM6DSO32 via Adafruit LSM6DS library)
#include <Adafruit_LSM6DSOX.h>

// Barometer (BMP390)
#include <Adafruit_BMP3XX.h>

// GPS (Arduino MKR GPS Module)
#include <Arduino_MKRGPS.h>

class SensorManager {
public:
  SensorManager(uint8_t lsmCSPin, uint8_t bmpCSPin);

  bool begin();
  void update();

  // ---------- IMU ----------
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;

  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;

  // ---------- Barometer ----------
  float temperature = 0.0f;     // °C
  float pressure_hPa = 0.0f;    // hPa
  float altitude_m = 0.0f;      // meters

  // ---------- GPS ----------
  bool  gpsFix = false;
  float latitude = 0.0f;
  float longitude = 0.0f;
  float gpsAltitude = 0.0f;
  int   satellites = 0;

private:
  uint8_t _lsmCSPin;
  uint8_t _bmpCSPin;

  Adafruit_LSM6DSOX lsm;
  Adafruit_BMP3XX   bmp;
};

