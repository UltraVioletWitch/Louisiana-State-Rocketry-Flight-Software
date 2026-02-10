#include <Arduino.h>
#include <future>
#include "AllSensors.h"

// GPS on Serial2, LSM CS=10, BMP CS=9
AllSensors sensors(Serial2, 9600, 10, 9);
enum State {
    PRE_LAUNCH,
    ASCENT,
    DESCENT,
    LANDED
};

State flightState = PRE_LAUNCH;

bool launchDetect(AllSensors);
bool apogeeDetect(AllSensors);
bool landingDetect(AllSensors);

void setup() {
    Serial.begin(115200);
    if (!sensors.begin()) {
        Serial.println(F("One or more sensors failed to initialize!"));
    }
}

void loop() {
    switch (flightState) {
        case PRE_LAUNCH:
            if (launchDetect(sensors)) {
                flightState = ASCENT;
                break;
            } else {
                break;
            }
        case ASCENT:
            if (apogeeDetect(sensors)) {
                flightState = DESCENT;
                break;
            } else {
                break;
            }
        case DESCENT:
            if (landingDetect(sensors)) {
                flightState = LANDED;
                break;
            } else {
                break;
            }
        case LANDED:
            break;
        default:
            break;
    }
}

bool launchDetect(AllSensors s) {
    return true;
}

bool apogeeDetect(AllSensors s) {
    return true;
}

bool landingDetect(AllSensors s) {
    return true;
}
