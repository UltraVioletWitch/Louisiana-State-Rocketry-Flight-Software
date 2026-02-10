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
LSR_Struct ring[RING_SIZE];

// detect function prototypes
bool launchDetect(LSR_Struct*, int, int);
bool burnoutDetect(LSR_Struct*, int, int);
bool apogeeDetect(LSR_Struct*, int, int);
bool landingDetect(LSR_Struct*, int, int);

void writeRing(float*, float*, int, int);
void writePacket();

void setup() {
    Serial.begin(115200);
    if (!sensors.begin()) {
        Serial.println(F("One or more sensors failed to initialize!"));
    }

    /* Setup code here */

    accelAltTimer = millis();
    GPSTimer = millis();
}

void loop() {
    switch (data.flightState) {
        case PRE_LAUNCH:
            if (launchDetect(&data, RING_SIZE, ringPtr)) {
                data.flightState = BURN;
                break;
            } else {
                /* Pre-Launch Code goes here */
                break;
            }
        case BURN:
            if (burnoutDetect(&data, RING_SIZE, ringPtr)) {
                data.flightState = COAST;
                break;
            } else {
                /* Burn code here */
                break;
            }
        case COAST:
            if (apogeeDetect(&data, RING_SIZE, ringPtr)) {
                data.flightState = DESCENT;
                break;
            } else {
                /* Coast code here */
                break;
            }
        case DESCENT:
            if (landingDetect(&data, RING_SIZE, ringPtr)) {
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
        if (ringPtr == RING_SIZE - 1) {
            ringPtr = 0;
        } else {
            ringPtr++;
        }
        ring[ringPtr] = data;
        accelAltTimer = millis();
    }

    if (millis() - GPSTimer > (1.0 / GPSTimer) * 1000) {
        /* GPS code */
        GPSTimer = millis();
    }
    
}

bool launchDetect(
        LSR_Struct *ring, 
        int ringSize, 
        int ringPtr) {
    int nextPtr;
    if (ringPtr == RING_SIZE - 1) {
        nextPtr = 0;
    } else {
        nextPtr = ringPtr + 1;
    }

    if (ring[ringPtr].AccelZ - ring[nextPtr].AccelZ > 10) {
        return true;
    }
    return false;
}

bool burnoutDetect(
        LSR_Struct *data, 
        int ringSize, 
        int ringPtr) {
    return true;
}

bool apogeeDetect(
        LSR_Struct *data, 
        int ringSize,
        int ringPtr) {
    return true;
}

bool landingDetect(
        LSR_Struct *data, 
        int ringSize,
        int ringPtr) {
    return true;
}
