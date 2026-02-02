#include <Arduino.h>
#include "AllSensors.h"

// GPS on Serial2, LSM CS=10, BMP CS=9
AllSensors sensors(Serial2, 9600, 10, 9);

void setup() {
    Serial.begin(115200);
    if (!sensors.begin()) {
        Serial.println(F("One or more sensors failed to initialize!"));
    }

    // Start BMP auto-calibration: discard 10 readings, average next 50
    sensors.startBMPSeaLevelAutoCalibration(50);
}

void loop() {
    sensors.update(); // prints parsed values every second

    // Optional: indicate calibration status
    if (!sensors.isBMPAutoCalibrated()) {
        Serial.println(F("BMP auto-calibration in progress..."));
    }
}

