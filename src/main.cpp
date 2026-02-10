#include "AllSensors.h"
#include "LSR_Struct.h"

// GPS on Serial2, LSM CS=10, BMP CS=9
AllSensors sensors(Serial2, 9600, 10, 9);
unsigned long accelAltTimer, GPSTimer;
const float accelAltHz = 100;
const float GPSHz = 10;

// data structure
LSR_Struct data;

// ring buffer
const int RING_SIZE = 8;
int ringPtr = 0;
float accelRing[RING_SIZE];
float altRing[RING_SIZE];

// detect function prototypes
bool launchDetect(LSR_Struct*, float*, float*, int, int);
bool apogeeDetect(LSR_Struct*, float*, float*, int, int);
bool landingDetect(LSR_Struct*, float*, float*, int, int);

void setup() {
    Serial.begin(115200);
    if (!sensors.begin()) {
        Serial.println(F("One or more sensors failed to initialize!"));
    }

    /* Setup code here */

    accelTimer = millis();
    altTimer = millis();
    GPSTimer = millis();
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
                /* Ascent code here */
                break;
            }
        case DESCENT:
            if (landingDetect(&data, altRing, accelRing, RING_SIZE, ringPtr)) {
                data.flightState = LANDED;
                break;
            } else {
                /* Descent code here */
                break;
            }
        case LANDED:
            /* Landed code here */
            break;
        default:
            delay(100);
            break;
    }

    /* Kalman Filter and Sensor Reading Code */
    if (millis() - accelAltTimer > (1.0 / accelAltHz) * 1000) {
        /* accel code */
        /* altimeter code */
        ringPtr++;
        accelRing[ringPtr] = data.AccelZ;
        altRing[ringPtr] = data.PosZ;
        accelAltTimer = millis();
    }

    if (millis() - GPSTimer > (1.0 / GPSTimer) * 1000) {
        /* GPS code */
        GPSTimer = millis();
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
