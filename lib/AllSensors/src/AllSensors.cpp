#include "AllSensors.h"
#include <math.h>
extern RingBuffer<RING_SIZE> ring;
// Constructor
AllSensors::AllSensors(HardwareSerial *gpsSerialHardware, uint32_t baud, int lsmCSPin, int bmpCSPin)
    : gpsSerialHardware(gpsSerialHardware), gpsBaud(baud), lsmCS(lsmCSPin), bmpCS(bmpCSPin) {}

AllSensors::AllSensors(SoftwareSerial *gpsSerialSoftware, uint32_t baud, int lsmCSPin, int bmpCSPin)
    : gpsSerialSoftware(gpsSerialSoftware), gpsBaud(baud), lsmCS(lsmCSPin), bmpCS(bmpCSPin) {}    
// Initialize sensors
bool AllSensors::begin() {
    bool success = true;
    // GPS
    if(gpsSerialHardware) {
        gpsSerialHardware->begin(gpsBaud);
        Serial.println(F("=== GPS Initialized ==="));
    } else if (gpsSerialSoftware) {
        gpsSerialSoftware->begin(gpsBaud);
        Serial.println(F("=== GPS Initialized ==="));
    } else {
        Serial.println(F("No GPS serial interface provided!"));
        success = false;
    }
    Serial.println(F("=== GPS Initialized ==="));

    // LSM6DSO32 SPI
    if (!lsm.begin_SPI(lsmCS)) {
        Serial.println(F("Failed to find LSM6DSO32 on SPI!"));
        success = false;
    } else {
        Serial.println(F("LSM6DSO32 OK"));
        
        lsm.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
        lsm.setAccelDataRate(LSM6DS_RATE_104_HZ);

        lsm.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
        lsm.setGyroDataRate(LSM6DS_RATE_104_HZ);
        // lsm.highPassFilter(true,   LSM6DS_HPF_ODR_DIV_50);

        lsm.configIntOutputs(true, false);
        lsm.configInt1(false, true, false);
        lsm.configInt2(false, false, true);
    }

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
    elapsedMillis timeBetweenPressureReadings;
    
    for (int i = 0; i < 10; i++) {
        if(timeBetweenPressureReadings > 100) {
            bmp.performReading();
            timeBetweenPressureReadings = 0;
        }
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

// Get the altitude from the BMP
const float AllSensors::getAltitudeBMP(void) {
    float bmpPressure_hPa = bmp.pressure / 100.0;
    float altitude_m = 44330.0 * (1.0 - pow(bmpPressure_hPa / bmpSeaLevel_hPa, 0.1903));
    return altitude_m;
}

const float AllSensors::getAltitudeBMP(float pressure_hPa) {
    float altitude_m = 44330.0 * (1.0 - pow(pressure_hPa / bmpSeaLevel_hPa, 0.1903));
    return altitude_m;
}

const float AllSensors::getSeaLevelPressure(void) {
    return bmpSeaLevel_hPa;
}

// Update all sensors
void AllSensors::update() {
    // --- GPS Update ---
    // while (gpsSerial.available() > 0) {
    //     gps.encode(gpsSerial.read());
    // }

    // --- IMU Update ---
    Serial.printf(F("Getting IMU Data\n"));
    if(lsm.getEvent(&accel, &gyro, &temp)) {
        Serial.println("IMU DATA");
        Serial.printf("x: %d,\t\ty: %d,\t\tz: %d\nx: %d,\t\ty: %d,\t\tz: %d\nTemp: %d", 
            accel.acceleration.x, 
            accel.acceleration.y, 
            accel.acceleration.z, 
            gyro.gyro.x, 
            gyro.gyro.y, 
            gyro.gyro.z, 
            temp.temperature
        );
    }

    // --- BMP Update ---
    Serial.printf(F("Getting BMP Data\n"));
    bmp.performReading();

    // --- Print all sensor data every 1 second ---
    // if (millis() - lastPrint > 1000) {
    //     lastPrint = millis();
    //     Serial.println(F("\n==== Sensor Readings ===="));

    //     // GPS
    //     Serial.print(F("GPS Fix: "));
    //     Serial.println(gps.location.isValid() ? "Yes" : "No");
    //     Serial.print(F("Latitude: "));
    //     Serial.println(gps.location.isValid() ? String(gps.location.lat(), 6) : "Invalid");
    //     Serial.print(F("Longitude: "));
    //     Serial.println(gps.location.isValid() ? String(gps.location.lng(), 6) : "Invalid");
    //     Serial.print(F("Altitude [m]: "));
    //     Serial.println(gps.altitude.isValid() ? String(gps.altitude.meters()) : "Invalid");
    //     Serial.print(F("Satellites: "));
    //     Serial.println(gps.satellites.isValid() ? String(gps.satellites.value()) : "Invalid");

    //     // LSM6DSO32 IMU
    //     Serial.print(F("Accel [m/s^2] X: ")); Serial.print(accel.acceleration.x);
    //     Serial.print(F(" Y: ")); Serial.print(accel.acceleration.y);
    //     Serial.print(F(" Z: ")); Serial.println(accel.acceleration.z);

    //     Serial.print(F("Gyro [rad/s] X: ")); Serial.print(gyro.gyro.x);
    //     Serial.print(F(" Y: ")); Serial.print(gyro.gyro.y);
    //     Serial.print(F(" Z: ")); Serial.println(gyro.gyro.z);

    //     // BMP390
    //     Serial.print(F("Pressure [Pa]: ")); Serial.println(bmp.pressure);
    //     Serial.print(F("Temperature [C]: ")); Serial.println(bmp.temperature);

    //     // Altitude calculation using calibrated sea-level pressure
    //     float bmpPressure_hPa = bmp.pressure / 100.0;
    //     float altitude_m = 44330.0 * (1.0 - pow(bmpPressure_hPa / bmpSeaLevel_hPa, 0.1903));
    //     Serial.print(F("Altitude [m]: ")); Serial.println(altitude_m);

    //     Serial.println(F("========================="));
    // }
}

void AllSensors::updateNoKalmanFilter(LSR_Struct& packet) {
    static size_t lastMicros;
    size_t currentMicros = micros();
    float deltaMicros = (currentMicros - lastMicros) / 1000000.0;

    if(deltaMicros <= 0) {
        deltaMicros = 0.0001;
    }

    // Adafruit Library returns gyro data in rad/s by default!!!
    lsm.getEvent(&accel, &gyro, &temp);

    // TODO: Handle gravity for the velocity calculation to prevent higher velocity readings
    if(abs(accel.acceleration.x) > accelBiasX) {
        velocityX += accel.acceleration.x * deltaMicros;
    }
    if(abs(accel.acceleration.y) > accelBiasY) {
        velocityY += accel.acceleration.y * deltaMicros;
    }
    if(abs(accel.acceleration.z) > accelBiasZ) {
        velocityZ += accel.acceleration.z * deltaMicros;
    }

    if(abs(gyro.gyro.x) > gyroBiasX) {
        gyroX += degrees(gyro.gyro.x) * deltaMicros;
    }
    if(abs(gyro.gyro.y) > gyroBiasY) {
        gyroY += degrees(gyro.gyro.y) * deltaMicros;
    }
    if(abs(gyro.gyro.z) > gyroBiasZ) {
        gyroZ += degrees(gyro.gyro.z) * deltaMicros;
    }

    lastMicros = currentMicros;

    bmp.performReading();

    if(gpsSerialHardware) {
        while(gpsSerialHardware->available()) {
            gps.encode(gpsSerialHardware->read());
        }
    } else if (gpsSerialSoftware) {
        while(gpsSerialSoftware->available()) {
            gps.encode(gpsSerialSoftware->read());
        }
    } else {
        Serial.println(F("No GPS serial interface provided!"));
    }

    if(gps.location.isValid()) {
        rocketLatitude = gps.location.lat();
        rocketLongitude = gps.location.lng();
    }

    if(gps.speed.isValid()) {
        rocketSpeed = gps.speed.mps();
    }

    packet = {
        millis(),
        accel.acceleration.x,
        accel.acceleration.y,
        accel.acceleration.z,
        gyro.gyro.x,
        gyro.gyro.y,
        gyro.gyro.z,
        velocityX,
        velocityY, 
        velocityZ,
        float(rocketLatitude),
        float(rocketLongitude),
        bmp.readAltitude(bmpSeaLevel_hPa), // This is giving weird results!
        gyroX, 
        gyroY, 
        gyroZ,
        float(bmp.pressure / 100),
        float(temp.temperature)
    };
}

void AllSensors::dataReadyLSMInt1(void) {
    lsmDataReadyInt1 = true;
}

void AllSensors::dataReadyLSMInt2(void) {
    lsmDataReadyInt2 = true;
}

const uint8_t AllSensors::getLSM6DSO32IntPin1(void) {
    return lsmInterruptPin1;
}

const uint8_t AllSensors::getLSM6DSO32IntPin2(void) {
    return lsmInterruptPin2;
}