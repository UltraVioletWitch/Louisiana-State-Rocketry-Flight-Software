#include "AllSensors.h"
#include <climits>
#include "LSR_Struct.h"
// #include "PID.h"
#include <RadioLib.h>
#include <SD.h>

#define __TEST__ 0

// GPS on Serial2, LSM CS=10, BMP CS=9
AllSensors sensors(Serial2, 9600, 24, 0);
unsigned long accelAltTimer, GPSTimer;
const float accelAltHz = 100;
const float GPSHz = 10;

// data structure
LSR_Struct data;
State currentFlightState = PRE_LAUNCH;

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

// SD card 
File loggingFile;
void writePacketToSD(const LSR_Struct& data);
void SDWriteTimerCallback();
IntervalTimer SDTimer;
const uint32_t SDWriteFreqMicroseconds = 100000;

// radio setup
const uint8_t radioCSPin = 7;
const uint8_t radioResetPin = 32;
const uint8_t radioBusyPin = 31;
const uint8_t radioDIO1Pin = 30;
const uint8_t radioDIO2Pin = 27;
const float EBYTE_FREQ = 912.3;
SX1262 radio = new Module(radioCSPin, radioDIO1Pin, radioResetPin, radioBusyPin);

// Servo Pins
constexpr uint8_t ServoPins[4] = {33, 36, 37, 14};

// Initalized variables 
bool SDcardPresent = false;
bool radioPresent = false;
bool writeToSD = false;

void setup() {
    Serial.begin(115200);
    #if __TEST__
        while(!Serial) {
            delay(200);
        }
    #endif

    if (!sensors.begin()) {
        Serial.println(F("One or more sensors failed to initialize!"));
    }

    // SPI.begin();

    /* Setup code here */
    // int16_t radioState = radio.begin(EBYTE_FREQ);
    // if(radioState != RADIOLIB_ERR_NONE) {
    //     Serial.printf("Failed to initialize radio, error code: %d\n", radioState);
    // } else {
    //     radioPresent = true;
    //     Serial.printf(F("EByte Initalized\n"));
    // }

    if(!SD.begin(BUILTIN_SDCARD)) {
        Serial.printf(F("Failed to initialize SD card!\n"));
    } else {
        SDcardPresent = true;

        if(!SDTimer.begin(SDWriteTimerCallback, SDWriteFreqMicroseconds)) {
            Serial.printf(F("SD Timer Failed\n"));
        }

        if (!SD.exists("/logs")) {
            SD.mkdir("/logs");
        }

        if(SD.exists("/logs/log.csv")) {
            uint16_t fileNumber = 1;
            while(SD.exists(("/logs/log" + String(fileNumber) + ".csv").c_str())) {
                fileNumber++;
            }
            loggingFile = SD.open(("/logs/log" + String(fileNumber) + ".csv").c_str(), FILE_WRITE | FILE_READ);
            Serial.printf("Created logging file: %s\n", ("/logs/log" + String(fileNumber) + ".csv").c_str());
        } else {
            loggingFile = SD.open("/logs/log.csv", FILE_WRITE | FILE_READ);
        }

        while(!loggingFile) {
            delayMicroseconds(2000);
        }
        loggingFile.println("TimeStamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,VelX,VelY,VelZ,PosX,PosY,PosZ,Theta,Phi,Psi,Pressure,FlightState");
        loggingFile.flush();
    }

    // Initalize the Servo Pins
    for (auto &&ServoNumber : ServoPins) {
        pinMode(ServoNumber, OUTPUT);
    }

    accelAltTimer = millis();
    GPSTimer = millis();
}

uint32_t loops;

void loop() { 
    
    #if __TEST__ // Set of commands that can allow us to make state changes to test the program at will
        if(Serial.available()) {
            String serialTestCommand = Serial.readString();
            if(serialTestCommand.equalsIgnoreCase("PRELAUNCH")) {
                Serial.printf(F("--PRELAUNCH--\n\n"));
                currentFlightState = PRE_LAUNCH;
            } else if(serialTestCommand.equalsIgnoreCase("BURN")) {
                Serial.printf(F("--BURN--\n\n"));
                currentFlightState = BURN;
            } else if(serialTestCommand.equalsIgnoreCase("COAST")) {
                Serial.printf(F("--COAST--\n\n"));
                currentFlightState = COAST;
            } else if(serialTestCommand.equalsIgnoreCase("LANDED")) {
                Serial.printf(F("--LANDED--\n\n"));
                currentFlightState = LANDED;
            } else if(serialTestCommand.equalsIgnoreCase("FLUSH")) {
                loggingFile.flush();
            } else {
                Serial.printf(F("Invalid Serial Command\n"));
            } 
        }
    #endif

    sensors.updateNoKalmanFilter(data);
    ring.push(data);

    switch (currentFlightState) {
        case PRE_LAUNCH: {
            if (launchDetect(ring)) {
                Serial.printf(F("--Burn--"));
                currentFlightState = BURN;
                launchTime = millis();
                /* code to log entire ring goes here */
                if(!SDcardPresent) {
                    break;
                }

                if (!loggingFile) {
                    break;
                }
                
                LSR_Struct preLaunchLoggingFile;

                // Log the entire ring buffer to the SD card.
                for(size_t i = 0; i < RING_SIZE; i++) {
                    preLaunchLoggingFile = ring[i];
                    loggingFile.printf("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n", 
                        millis(), 
                        preLaunchLoggingFile.AccelX, 
                        preLaunchLoggingFile.AccelY, 
                        preLaunchLoggingFile.AccelZ, 
                        preLaunchLoggingFile.GyroX, 
                        preLaunchLoggingFile.GyroY, 
                        preLaunchLoggingFile.GyroZ, 
                        preLaunchLoggingFile.VelX, 
                        preLaunchLoggingFile.VelY, 
                        preLaunchLoggingFile.VelZ, 
                        preLaunchLoggingFile.PosX, 
                        preLaunchLoggingFile.PosY, 
                        preLaunchLoggingFile.PosZ, 
                        preLaunchLoggingFile.Theta, 
                        preLaunchLoggingFile.Phi, 
                        preLaunchLoggingFile.Psi, 
                        preLaunchLoggingFile.Pressure, 
                        currentFlightState
                    );
                }

                loggingFile.flush();

                // Increase the writing frequency to the SD card during flight
                SDTimer.update(SDWriteFreqMicroseconds / 10.0);

                break;
            } else {
                /* Pre-Launch Code goes here */
                writePacketToSD(ring.getFirst());
                break;
            }
        }
        case BURN: {
            if (burnoutDetect(ring)) {
                Serial.printf(F("--Burnout--"));
                currentFlightState = COAST;
                burnTime = millis();
                loggingFile.flush();
                break;
            } else {
                /* Burn code here */
                writePacketToSD(ring.getFirst());
                break;
            }
        }
        case COAST: {
            if (apogeeDetect(ring)) {
                Serial.printf(F("--Descent--"));
                currentFlightState = DESCENT;
                apogeeTime = millis();
                loggingFile.flush();
                break;
            } else {
                /* Coast code here */
                writePacketToSD(ring.getFirst());
                break;
            }
        }
        case DESCENT: {
            if (landingDetect(ring)) {
                Serial.printf(F("--Landed--"));
                currentFlightState = LANDED;
                landTime = millis();
                loggingFile.flush();
                break;
            } else {
                /* Descent code here */
                writePacketToSD(ring.getFirst());
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
            if(radioPresent) {
                // int16_t landingTransmitStatus = radio.startTransmit((const uint8_t*)&data, sizeof(data));
                // if (landingTransmitStatus != RADIOLIB_ERR_NONE) {
                //     Serial.printf("Failed to start transmission, error code: %d\n", landingTransmitStatus);
                // }
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
    #if __TEST__
        const uint8_t accelThreshold = 12;
        const uint8_t samplesRequired = 50;
        const uint32_t altimeterThreshold = 0;

    #else
        const Acceleration accelThreshold = Acceleration::G_9;
        const uint8_t samplesRequired = 100;
        const uint32_t altimeterThreshold = 2;
    #endif
    
    static uint8_t accelCount;
    constexpr auto maxAccelCount = std::numeric_limits<decltype(accelCount)>::max();

    if(averageAccelZ > accelThreshold) {
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
    #if __TEST__
        const uint8_t accelThreshold = 10;
        const uint32_t altimeterThreshold = 0;
        const uint8_t samplesRequired = 50;
    #else
        const Acceleration accelThreshold = Acceleration::G_9;
        const uint32_t altimeterThreshold = 2;
        const uint8_t samplesRequired = 100;
    #endif
    static uint8_t accelCount;
    constexpr auto maxAccelCount = std::numeric_limits<decltype(accelCount)>::max();

    // Check if the average vertical acceleration is below the launch G-forces
    if(averageAccelZ < accelThreshold) {
        accelCount + 1 > maxAccelCount ? accelCount = maxAccelCount : accelCount++;
        printf("BA: %d", accelCount);
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

    // Setup the requirements for burnout detection
    #if __TEST__
        const uint32_t altimeterThreshold = 1;
        const uint8_t velocityThreshold = 0;
        const uint8_t samplesRequired = 50;
    #else
        const uint32_t altimeterThreshold = 5;
        const uint8_t velocityThreshold = 2;
        const uint8_t samplesRequired = 100;
    #endif
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
    return false;
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

void writePacketToSD(const LSR_Struct& data) {
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
        loggingFile.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
            millis(), 
            data.AccelX, 
            data.AccelY, 
            data.AccelZ, 
            data.GyroX, 
            data.GyroY, 
            data.GyroZ, 
            data.VelX, 
            data.VelY, 
            data.VelZ, 
            data.PosX, 
            data.PosY, 
            data.PosZ, 
            data.Theta, 
            data.Phi, 
            data.Psi, 
            data.Pressure, 
            currentFlightState
        );

        // Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
        //     millis(), 
        //     data.AccelX, 
        //     data.AccelY, 
        //     data.AccelZ, 
        //     data.GyroX, 
        //     data.GyroY, 
        //     data.GyroZ, 
        //     data.VelX, 
        //     data.VelY, 
        //     data.VelZ, 
        //     data.PosX, 
        //     data.PosY, 
        //     data.PosZ, 
        //     data.Theta, 
        //     data.Phi, 
        //     data.Psi, 
        //     data.Pressure, 
        //     data.flightState
        // );
    } else {
        Serial.println(F("Failed to write to SD card!"));
    }

    writeToSD = false;
    interrupts();
}