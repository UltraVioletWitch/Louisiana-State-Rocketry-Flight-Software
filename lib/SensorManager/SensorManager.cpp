#include "SensorManager.h"

SensorManager::SensorManager(uint8_t lsmCSPin, uint8_t bmpCSPin)
  : _lsmCSPin(lsmCSPin),
    _bmpCSPin(bmpCSPin) {}

bool SensorManager::begin() {
  // ---------- SPI ----------
  SPI.begin();

  pinMode(_lsmCSPin, OUTPUT);
  pinMode(_bmpCSPin, OUTPUT);
  digitalWrite(_lsmCSPin, HIGH);
  digitalWrite(_bmpCSPin, HIGH);

  // ---------- LSM6DSOX (SPI) ----------
  if (!lsm.begin_SPI(_lsmCSPin)) {
    Serial.println("ERROR: LSM6DSOX SPI init failed");
    return false;
  }

  lsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  lsm.setAccelDataRate(LSM6DS_RATE_833_HZ);
  lsm.setGyroDataRate(LSM6DS_RATE_833_HZ);

  // ---------- BMP390 (SPI) ----------
  if (!bmp.begin_SPI(_bmpCSPin)) {
    Serial.println("ERROR: BMP390 SPI init failed");
    return false;
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // ---------- GPS ----------
  if (!GPS.begin()) {
    Serial.println("ERROR: GPS init failed");
    return false;
  }

  return true;
}

void SensorManager::update() {
  // ---------- IMU ----------
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  if (lsm.getEvent(&accel, &gyro, &temp)) {
    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;

    gx = gyro.gyro.x;
    gy = gyro.gyro.y;
    gz = gyro.gyro.z;
  }

  // ---------- BMP390 ----------
  if (bmp.performReading()) {
    temperature  = bmp.temperature;
    pressure_hPa = bmp.pressure / 100.0f;
    altitude_m   = bmp.readAltitude(1013.25f); // sea-level reference
  }

  // ---------- GPS ----------
  gpsFix = GPS.available();

  if (gpsFix) {
      latitude    = GPS.latitude();
      longitude   = GPS.longitude();
      gpsAltitude = GPS.altitude();
      satellites  = GPS.satellites();
  }

}

