#include "AllSensors.h"
#include "LSR_Struct.h"
#include <RadioLib.h>

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

// radio setup
SX1262 radio = new Module(SCK, MISO, MOSI, 7);
const float EBYTE_FREQ = 912.3;


void writePacket();

void setup() {
    Serial.begin(115200);
    if (!sensors.begin()) {
        Serial.println(F("One or more sensors failed to initialize!"));
    }

    /* Setup code here */
    radio.begin(EBYTE_FREQ);

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
            int16_t landingTransmitStatus = radio.startTransmit((uint8_t*)&data, sizeof(data));
            if (landingTransmitStatus != RADIOLIB_ERR_NONE) {
                Serial.printf("Failed to start transmission, error code: %d\n", landingTransmitStatus);
            }
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
    // Use bitwise operation to track the accepted thresholds for each sensor, 
    // and only return true if all thresholds are passed within a certain time frame.
    // This is to prevent false positives from a single sensor.
    static uint8_t trackingBit;
    static uint64_t accelThreshold;
    static uint64_t bmpThreshold;
    

    data = ring.getFirst();
    Serial.print(data.PosZ);

    // Check if the current acceleration exceeds the threshold for launch.
    // If it does, set the corresponding bit in accelThreshold.
    // If it isn't, clear the threshold.
    if(data.VelZ > Acceleration::G_14) {
        accelThreshold |= (1 << trackingBit);
    } else {
        accelThreshold = 0;
    }

    // Check if the current pressure is below the threshold for launch.
    // If it is, set the corresponding bit in bmpThreshold.
    // If it isn't, clear the threshold.
    if(data.Pressure < ring.getLast().Pressure) {
        bmpThreshold |= (1 << trackingBit);
    } else {
        bmpThreshold = 0;
    }

    // If either threshold is not met, reset the all variables and return false.
    if(!accelThreshold || !bmpThreshold) {
        trackingBit = 0;
        accelThreshold = 0;
        bmpThreshold = 0;
        return false;
    }

    if(accelThreshold == UINT64_MAX && bmpThreshold == UINT64_MAX) {
        trackingBit = 0;
        return true;
    }
    
    trackingBit++;
    return false;
}

bool burnoutDetect(const RingBuffer<RING_SIZE>& ring) {
    return true;
}

bool apogeeDetect(const RingBuffer<RING_SIZE>& ring) {
    return true;
}

bool landingDetect(const RingBuffer<RING_SIZE>& ring) {
    data = ring.getFirst();

    // Check if the current acceleration for x,y,z exceeds the threshold for landing.
    // If it does, return false.
    if(data.VelX > Acceleration::G_1 && data.VelY > Acceleration::G_1 && data.VelZ > Acceleration::G_1) {
        return false;
    }

    // Check if the current position for the z-axis is within the threshold for landing.
    // If it isn't, return false.
    if(abs(data.PosZ - ring.getLast().PosZ) > 3) {
        return false;
    }

    // Check if the current pressure is within the threshold for landing.
    // If it isn't, return false.
    if(abs(data.Pressure - ring.getLast().Pressure) < 5) {
        return false;
    }

    return true;
}
