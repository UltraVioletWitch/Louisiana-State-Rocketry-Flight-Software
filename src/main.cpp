#include "AllSensors.h"
#include "LSR_Struct.h"
#include <RadioLib.h>
#include <SD.h>
#include <climits>

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
const uint32_t SDWriteFreqMicroseconds = 100000;

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

// BMP390
uint16_t seaLevelAltitude = 0;

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

    seaLevelAltitude = sensors.getAltitudeBMP();

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
        case PRE_LAUNCH: {
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
        }
        case BURN: {
            if (burnoutDetect(ring)) {
                data.flightState = COAST;
                burnTime = millis();
                break;
            } else {
                /* Burn code here */
                break;
            }
        }
        case COAST: {
            if (apogeeDetect(ring)) {
                data.flightState = DESCENT;
                apogeeTime = millis();
                break;
            } else {
                /* Coast code here */
                break;
            }
        }
        case DESCENT: {
            if (landingDetect(ring)) {
                data.flightState = LANDED;
                landTime = millis();
                break;
            } else {
                /* Descent code here */
                break;
            }
        }
        case LANDED: {
            
            // Close the SD card, it is no longer needed and having it open risk corruption
            if(SDcardPresent) {
                loggingFile.flush();
                loggingFile.close();
                SDcardPresent = false;
            }
            
            // Transmit the position of where the rocket landed
            int16_t landingTransmitStatus = radio.startTransmit((const uint8_t*)&data, sizeof(data));
            if (landingTransmitStatus != RADIOLIB_ERR_NONE) {
                Serial.printf("Failed to start transmission, error code: %d\n", landingTransmitStatus);
            }
            break;
        }
        default: {
            delay(100);
            break;
        }
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
    // Initialize variables for launch detection
    float averageAccelZ = 0;
    float averagePressure = 0;
    static float prevAverageAccelZ;
    static float prevAveragePressureToAltitude;
    bool altitudeIncreasing = false;

    // Get the values from the current ring buffer and average them
    for(uint8_t index = 0; index < RING_SIZE; index++) {
        averageAccelZ += ring[index].AccelZ;
        averagePressure += ring[index].Pressure;
    }
    averageAccelZ /= RING_SIZE;
    averagePressure /= RING_SIZE;
    const float averagePressureToAltitude = sensors.getAltitudeBMP(averagePressure);

    // Get the differntials from the previous averages
    float averageAccelZDifferential = averageAccelZ - prevAverageAccelZ;
    float averagePressureAltitudeDifferential = averagePressureToAltitude - prevAveragePressureToAltitude;

    // Update the previous averages for the next burnout detection
    prevAverageAccelZ = averageAccelZ;
    prevAveragePressureToAltitude = averagePressureToAltitude;

    // Save the current sensor requirements for lauch detection
    const Acceleration accelThreshold = Acceleration::G_9;
    const uint8_t samplesRequired = 100;
    const uint32_t altimeterThreshold = 5;
    static uint8_t accelCount;
    constexpr auto maxAccelCount = std::numeric_limits<decltype(accelCount)>::max();

    if(averageAccelZ < accelThreshold) {
        accelCount + 1 > maxAccelCount ? accelCount = maxAccelCount : accelCount++;
    } else {
        accelCount = 0;
        return false;
    }

    // Check if the altitude is increasing
    if(averagePressureAltitudeDifferential >= altimeterThreshold) {
        altitudeIncreasing = true;
    } else {
        altitudeIncreasing = false;
    }

    if((accelCount >= samplesRequired) && altitudeIncreasing) {
        return true;
    }
    
    return false;
}

bool burnoutDetect(const RingBuffer<RING_SIZE>& ring) {
    // Initalize variables for burnout detection
    float averageAccelZ = 0;
    float averageVelZ = 0;
    float averagePressure = 0;
    static float prevAverageAccelZ;
    static float prevAverageVelZ;
    static float prevAveragePressureToAltitude;
    bool velocityDecreasing = false;
    bool altitudeIncreasing = false;

    // Get the values from the current ring buffer and average them
    for(uint8_t index = 0; index < RING_SIZE; index++) {
        averageAccelZ += ring[index].AccelZ;
        averageVelZ += ring[index].VelZ;
        averagePressure += ring[index].Pressure;
    }
    averageAccelZ /= RING_SIZE;
    averageVelZ /= RING_SIZE;
    averagePressure /= RING_SIZE;
    const float averagePressureToAltitude = sensors.getAltitudeBMP(averagePressure);

    // Get the differntials from the previous averages
    float averageAccelZDifferential = averageAccelZ - prevAverageAccelZ;
    float averageVelZDifferential = averageVelZ - prevAverageVelZ;
    float averagePressureAltitudeDifferential = averagePressureToAltitude - prevAveragePressureToAltitude;

    // Update the previous averages for the next burnout detection
    prevAverageAccelZ = averageAccelZ;
    prevAverageVelZ = averageVelZ;
    prevAveragePressureToAltitude = averagePressureToAltitude;

    // Setup the requirements for burnout detection
    const Acceleration accelThreshold = Acceleration::G_9;
    const uint32_t altimeterThreshold = 5;
    const uint8_t samplesRequired = 100;
    static uint8_t accelCount;
    constexpr auto maxAccelCount = std::numeric_limits<decltype(accelCount)>::max();

    // Check if the average vertical acceleration is below the launch G-forces
    if(averageAccelZ < accelThreshold) {
        accelCount + 1 > maxAccelCount ? accelCount = maxAccelCount : accelCount++;
    } else {
        accelCount = 0;
        return false;
    }

    // Check if the velocity is decreasing
    if(averageVelZDifferential <= 0) {   
        velocityDecreasing = true;
    } else {
        velocityDecreasing = false;
    }

    // Check if the altitude is increasing
    if(averagePressureAltitudeDifferential >= altimeterThreshold) {
        altitudeIncreasing = true;
    } else {
        altitudeIncreasing = false;
    }

    if((accelCount >= samplesRequired) && velocityDecreasing && altitudeIncreasing) {
        return true;
    }

    return false;
}

bool apogeeDetect(const RingBuffer<RING_SIZE>& ring) {
    // Initalize variables for apogee detection
    float averageVelZ = 0;
    float averagePressure = 0;
    static float prevAverageVelZ;
    static float prevAveragePressureToAltitude;
    static float lowestVelocity;
    static float highestAltitude;

    // Get the values from the current ring buffer and average them
    for(uint8_t index = 0; index < RING_SIZE; index++) {
        averageVelZ += ring[index].VelZ;
        averagePressure += ring[index].Pressure;
    }
    averageVelZ /= RING_SIZE;
    averagePressure /= RING_SIZE;
    const float averagePressureToAltitude = sensors.getAltitudeBMP(averagePressure);

    // Get the differentials from the previous averages
    float averageVelZDifferential = averageVelZ - prevAverageVelZ;
    float averagePressureAltitudeDifferential = averagePressureToAltitude - prevAveragePressureToAltitude;

    // Update the previous averages for the next apogee detection
    prevAverageVelZ = averageVelZ;
    prevAveragePressureToAltitude = averagePressureToAltitude;

    const uint32_t altimeterThreshold = 5;
    const uint8_t velocityThreshold = 5;
    const uint8_t samplesRequired = 10;
    static uint8_t passedSamples;
    constexpr auto maxSampleCount = std::numeric_limits<decltype(passedSamples)>::max();

    if((averageVelZDifferential >= velocityThreshold) && (averagePressureAltitudeDifferential <= altimeterThreshold)) {
        passedSamples + 1 > maxSampleCount ? passedSamples = maxSampleCount : passedSamples++;
    } else {
        passedSamples = 0;
    }

    if(passedSamples >= samplesRequired) {
        return true;
    }

    return false;
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
        interrupts();
        return;
    }

    if (!SDcardPresent) {
        interrupts();
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