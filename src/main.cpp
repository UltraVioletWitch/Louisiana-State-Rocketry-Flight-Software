#include "AllSensors.h"
#include "LSR_Struct.h"
#include <RadioLib.h>
#include <SD.h>

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
IntervalTimer SDTimer;
const uint32_t SDWriteFreqMicroseconds = 10000;

// SD card 
File loggingFile;
void writePacketToSD(const RingBuffer<RING_SIZE>& ring);
void SDWriteTimerCallback();

// radio setup
SX1262 radio = new Module(SCK, MISO, MOSI, 7);
const uint8_t radioResetPin = 32;
const uint8_t radioBusyPin = 31;
const uint8_t radioDIO1Pin = 30;
const uint8_t radioDIO2Pin = 27;
const float EBYTE_FREQ = 912.3;

// Initalized variables 
bool SDcardPresent = false;
bool radioPresent = false;
bool writeToSD = false;

void setup() {
    Serial.begin(115200);
    if (!sensors.begin()) {
        Serial.println(F("One or more sensors failed to initialize!"));
    }

    /* Setup code here */
    int16_t radioState = radio.begin(EBYTE_FREQ);
    if(radioState != RADIOLIB_ERR_NONE) {
        Serial.printf("Failed to initialize radio, error code: %d\n", radioState);
    } else {
        radioPresent = true;
    }

    if(!SD.begin(BUILTIN_SDCARD)) {
        Serial.printf(F("Failed to initialize SD card!\n"));
    } else {
        SDcardPresent = true;
        if (!SD.exists("/logs")) {
            SD.mkdir("/logs");
        }
        loggingFile = SD.open("/logs/log.txt", FILE_WRITE | FILE_READ);
    }

    // Create a timer that will set a flag to write to the SD card at a given frequency
    // I am using this for testing. It's also non-blocking, so it won't interfere with the main loop.
    if(SDTimer.begin(SDWriteTimerCallback, SDWriteFreqMicroseconds)) {
        Serial.println(F("SD write timer initialized successfully."));
    } else {
        Serial.println(F("Failed to initialize SD write timer."));
    }

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

                if(!SDcardPresent) {
                    break;
                }

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
            
            // Close the SD card, it is no longer needed and having it open risk corruption
            if(SDcardPresent) {
                loggingFile.flush();
                loggingFile.close();
                SDcardPresent = false;
            }
            
            // Transmit the position of where the rocket landed
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
    // Save the current sensor requirements for lauch detection
    const Acceleration accelThreshold = Acceleration::G_3;
    const uint8_t samplesRequired = 100;
    const uint8_t altimeterThreshold = 5;

    static uint8_t accelCount;
    data = ring.getFirst();
    
    // Get the current data from the Struct
    float currentAccelX = data.AccelX;
    float currentAccelY = data.AccelY;
    float currentAccelZ = data.AccelZ;
    float currentAltitudeBMP = sensors.getAltitudeBMP() - sensors.getSeaLevelPressure();

    // Check if the vertical acceleration is above the threshold, 
    // also check if the other axes are not too high to prevent horizontal movement from triggering launch detection.
    if(currentAccelZ >= accelThreshold && (currentAccelX < accelThreshold || currentAccelY < accelThreshold)) {
        accelCount++;
    } else {
        accelCount = 0;
        return false;
    }
    
    // TODO: Add a check for so that the angles from the acelerometer is no greater than 45 degrees.
    // The rocket will launch fairly verically

    // Check if we have enough samples from the accelerometer,
    // also check if the altimter is above the height threshold to prevent false positives from the accelerometer.
    if(accelCount >= samplesRequired && currentAltitudeBMP > altimeterThreshold) {
        return true;
    }
    
    return false;
}

bool burnoutDetect(const RingBuffer<RING_SIZE>& ring) {
    return true;
}

bool apogeeDetect(const RingBuffer<RING_SIZE>& ring) {
    return true;
}

bool landingDetect(const RingBuffer<RING_SIZE>& ring) {
    const Acceleration accelThreshold = Acceleration::G_1;
    const float velocityThreshold = 1.0;
    const float positionThreshold = 3.0;
    const float pressureThreshold = 5.0;
    
    data = ring.getFirst();

    // Check if the current acceleration for x,y,z exceeds the threshold for landing.
    // If it does, return false.
    if(data.AccelX > accelThreshold || data.AccelY > accelThreshold || data.AccelZ > accelThreshold) {
        return false;
    }

    // Check if the current velocity for x,y,z exceeds the threshold for landing.
    // If it does, return false.
    if(abs(data.VelX -  ring.getLast().VelX) > velocityThreshold || abs(data.VelY - ring.getLast().VelY) > velocityThreshold || abs(data.VelZ - ring.getLast().VelZ) > velocityThreshold) {
        return false;
    }

    // Check if the current position for the z-axis is within the threshold for landing.
    // If it isn't, return false.
    if(abs(data.PosZ - ring.getLast().PosZ) > positionThreshold) {
        return false;
    }

    // Check if the current pressure is within the threshold for landing.
    // If it isn't, return false.
    if(abs(data.Pressure - sensors.getSeaLevelPressure()) > pressureThreshold) {
        return false;
    }

    return true;
}

void SDWriteTimerCallback() {
    writeToSD = true;
}

void writePacketToSD(const RingBuffer<RING_SIZE>& ring) {
    noInterrupts();
    if(!writeToSD) {
        return;
    }

    if (!SDcardPresent) {
        return;
    }

    if (loggingFile) {
        loggingFile.write((const uint8_t*)&ring, sizeof(ring));
    } else {
        Serial.println(F("Failed to write to SD card!"));
    }

    writeToSD = false;
    interrupts();
}