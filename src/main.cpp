#include "AllSensors.h"
#include "LSR_Struct.h"

// GPS on Serial2, LSM CS=10, BMP CS=9
AllSensors sensors(Serial2, 9600, 10, 9);
LSR_Struct data;

bool launchDetect(LSR_Struct);
bool apogeeDetect(LSR_Struct);
bool landingDetect(LSR_Struct);

void setup() {
    Serial.begin(115200);
    if (!sensors.begin()) {
        Serial.println(F("One or more sensors failed to initialize!"));
    }
}

void loop() {
    switch (data.flightState) {
        case PRE_LAUNCH:
            if (launchDetect(data)) {
                data.flightState = ASCENT;
                break;
            } else {
                break;
            }
        case ASCENT:
            if (apogeeDetect(data)) {
                data.flightState = DESCENT;
                break;
            } else {
                break;
            }
        case DESCENT:
            if (landingDetect(data)) {
                data.flightState = LANDED;
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

bool launchDetect(LSR_Struct data) {
    return true;
}

bool apogeeDetect(LSR_Struct data) {
    return true;
}

bool landingDetect(LSR_Struct data) {
    return true;
}
