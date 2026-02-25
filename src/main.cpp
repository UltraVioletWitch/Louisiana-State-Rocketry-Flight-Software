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
static RingBuffer<RING_SIZE> ring;

// detect function prototypes
bool launchDetect(const RingBuffer<RING_SIZE>&);
bool burnoutDetect(const RingBuffer<RING_SIZE>&);
bool apogeeDetect(const RingBuffer<RING_SIZE>&);
bool landingDetect(const RingBuffer<RING_SIZE>&);

// time logs
unsigned long launchTime;
unsigned long burnTime;
unsigned long apogeeTime;
unsigned long landTime;


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
            if (launchDetect(ring)) {
                data.flightState = BURN;
                launchTime = millis();
                /* code to log entire ring goes here */
                break;
            } else {
                /* Pre-Launch Code goes here */
                break;
            }
        case BURN:
            if (burnoutDetect(ring)) {
                data.flightState = COAST;
                burnTime = millis();
                break;
            } else {
                /* Burn code here */
                break;
            }
        case COAST:
            if (apogeeDetect(ring)) {
                data.flightState = DESCENT;
                apogeeTime = millis();
                break;
            } else {
                /* Coast code here */
                break;
            }
        case DESCENT:
            if (landingDetect(ring)) {
                data.flightState = LANDED;
                landTime = millis();
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
        accelAltTimer = millis();
    }

    if (millis() - GPSTimer > (1.0 / GPSTimer) * 1000) {
        /* GPS code */
        GPSTimer = millis();
    }
}

bool launchDetect(const RingBuffer<RING_SIZE>& ring) {
    data = ring.getFirst();
    Serial.print(data.PosZ);
    return true;
}

bool burnoutDetect(const RingBuffer<RING_SIZE>& ring) {
    return true;
}

bool apogeeDetect(const RingBuffer<RING_SIZE>& ring) {
    return true;
}

bool landingDetect(const RingBuffer<RING_SIZE>& ring) {
    return true;
}
