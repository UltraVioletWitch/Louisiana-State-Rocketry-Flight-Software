#include "AllSensors.h"
#include "LSR_Struct.h"

// GPS on Serial2, LSM CS=10, BMP CS=9
AllSensors sensors(Serial2, 9600, 10, 9);
LSR_Struct data;

const int RING_SIZE = 8;
int ringPtr = 0;
float accelRing[RING_SIZE];
float altRing[RING_SIZE];

bool launchDetect(LSR_Struct*, float*, float*, int, int);
bool apogeeDetect(LSR_Struct*, float*, float*, int, int);
bool landingDetect(LSR_Struct*, float*, float*, int, int);

void setup() {
    Serial.begin(115200);
    if (!sensors.begin()) {
        Serial.println(F("One or more sensors failed to initialize!"));
    }
}

void loop() {
    switch (data.flightState) {
        case PRE_LAUNCH:
            if (launchDetect(&data, altRing, accelRing, RING_SIZE, ringPtr)) {
                data.flightState = ASCENT;
                break;
            } else {
                /* Pre-Launch Code goes here */
                break;
            }
        case ASCENT:
            if (apogeeDetect(&data, altRing, accelRing, RING_SIZE, ringPtr)) {
                data.flightState = DESCENT;
                break;
            } else {
                break;
            }
        case DESCENT:
            if (landingDetect(&data, altRing, accelRing, RING_SIZE, ringPtr)) {
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

bool launchDetect(
        LSR_Struct *data, 
        float *altRing, 
        float *accelRing, 
        int ringSize, 
        int ringPtr) {
    return true;
}

bool apogeeDetect(
        LSR_Struct *data, 
        float *altRing, 
        float *accelRing, 
        int ringSize,
        int ringPtr) {
    return true;
}

bool landingDetect(
        LSR_Struct *data, 
        float *altRing, 
        float *accelRing, 
        int ringSize,
        int ringPtr) {
    return true;
}
