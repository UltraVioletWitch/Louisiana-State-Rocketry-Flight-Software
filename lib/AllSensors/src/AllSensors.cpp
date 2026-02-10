#include "AllSensors.h"
#include <math.h>

// Constructor
AllSensors::AllSensors(HardwareSerial &serial, uint32_t baud, int lsmCSPin, int bmpCSPin)
    : gpsSerial(serial), gpsBaud(baud), lsmCS(lsmCSPin), bmpCS(bmpCSPin) {}

// Initialize sensors
bool AllSensors::begin() {
    bool success = true;

    // GPS
    gpsSerial.begin(gpsBaud);
    Serial.println(F("=== GPS Initialized ==="));

    // LSM6DSO32 SPI
    if (!lsm.begin_SPI(lsmCS)) {
        Serial.println(F("Failed to find LSM6DSO32 on SPI!"));
        success = false;
    } else Serial.println(F("LSM6DSO32 OK"));

    // BMP390 SPI
    if (!bmp.begin_SPI(bmpCS)) {
        Serial.println(F("Failed to find BMP390 on SPI!"));
        success = false;
    } else {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        Serial.println(F("BMP390 OK"));
    }

    calibrateBMPSeaLevel();

    return success;
}

// Manually calibrate BMP sea-level pressure
void AllSensors::calibrateBMPSeaLevel(void) {
    for (int i = 0; i < 10; i++) {
        bmp.performReading();
        delay(100);
    }

    float sumPressure = 0;
    int numReadings = 50;
    for (int i = 0; i < numReadings; i++) {
        bmp.performReading();
        sumPressure += bmp.pressure / 100.0;
    }
    bmpSeaLevel_hPa = sumPressure / numReadings;
    Serial.print(F("BMP sea-level pressure manually set to: "));
    Serial.println(bmpSeaLevel_hPa);
}

// Update all sensors
//
void AllSensors::readBMP() {
    bmp.performReading();

    pressure = bmp.pressure;
    temp = bmp.temperature;
}

void AllSensors::readIMU() {
    sensors_event_t accel, gyro, temp;
    lsm.getEvent(&accel, &gyro, &temp);

    AccelX = accel.acceleration.x;
    AccelY = accel.acceleration.y;
    AccelZ = accel.acceleration.z;
    gyroX = gyro.gyro.x;
    gyroY = gyro.gyro.y;
    gyroZ = gyro.gyro.z;
}

void AllSensors::readGPS() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    isValid = gps.location.isValid();
    lat = gps.location.lat();
    lon = gps.location.lng();
    alt = gps.altitude.meters();
}   


void AllSensors::update() {
    // --- GPS Update ---
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    // --- IMU Update ---
    sensors_event_t accel, gyro, temp;
    lsm.getEvent(&accel, &gyro, &temp);

    // --- BMP Update ---
    bmp.performReading();

    // --- Print all sensor data every 1 second ---
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        Serial.println(F("\n==== Sensor Readings ===="));

        // GPS
        Serial.print(F("GPS Fix: "));
        Serial.println(gps.location.isValid() ? "Yes" : "No");
        Serial.print(F("Latitude: "));
        Serial.println(gps.location.isValid() ? String(gps.location.lat(), 6) : "Invalid");
        Serial.print(F("Longitude: "));
        Serial.println(gps.location.isValid() ? String(gps.location.lng(), 6) : "Invalid");
        Serial.print(F("Altitude [m]: "));
        Serial.println(gps.altitude.isValid() ? String(gps.altitude.meters()) : "Invalid");
        Serial.print(F("Satellites: "));
        Serial.println(gps.satellites.isValid() ? String(gps.satellites.value()) : "Invalid");

        // LSM6DSO32 IMU
        Serial.print(F("Accel [m/s^2] X: ")); Serial.print(accel.acceleration.x);
        Serial.print(F(" Y: ")); Serial.print(accel.acceleration.y);
        Serial.print(F(" Z: ")); Serial.println(accel.acceleration.z);

        Serial.print(F("Gyro [rad/s] X: ")); Serial.print(gyro.gyro.x);
        Serial.print(F(" Y: ")); Serial.print(gyro.gyro.y);
        Serial.print(F(" Z: ")); Serial.println(gyro.gyro.z);

        // BMP390
        Serial.print(F("Pressure [Pa]: ")); Serial.println(bmp.pressure);
        Serial.print(F("Temperature [C]: ")); Serial.println(bmp.temperature);

        // Altitude calculation using calibrated sea-level pressure
        float bmpPressure_hPa = bmp.pressure / 100.0;
        float altitude_m = 44330.0 * (1.0 - pow(bmpPressure_hPa / bmpSeaLevel_hPa, 0.1903));
        Serial.print(F("Altitude [m]: ")); Serial.println(altitude_m);

        Serial.println(F("========================="));
    }
}

