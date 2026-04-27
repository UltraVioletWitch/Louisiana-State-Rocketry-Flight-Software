#include "AllSensors.h"
#include <climits>
#include "LSR_Struct.h"
#include "PID.h"
#include <RadioLib.h>
#include <SdFat.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <RingBuf.h>

#include "constants.h"

// GPS on Serial2, LSM CS=24, BMP CS=0
SoftwareSerial gpsSerial(CORE_RXD7_PIN, CORE_TXD7_PIN);
AllSensors sensors(&gpsSerial, 9600, 24, 0);
static const SPISettings spiSettings(1000000UL, MSBFIRST, SPI_MODE0); // What the default adafruit sensors use for SPI settings
unsigned long accelAltTimer, GPSTimer;
const float accelAltHz = 100;
const float GPSHz = 10;
void changeIMUInterruptPin1(void);
void changeIMUInterruptPin2(void);

// data structure
LSR_Struct data;
State currentFlightState = PRE_LAUNCH;

// ring buffer
RingBuffer<RING_SIZE> ring;

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
SdFs sdCard;
FsFile sdFile;
RingBuf<FsFile, 2 << 9> sdBuffer;
void writePacketToSD(const LSR_Struct& data);
void SDWriteTimerCallback();
IntervalTimer SDTimer;
const uint32_t SDWriteFreqMicroseconds = 100000;
elapsedMillis SDWriteElapsedTime;

// radio setup
// There is a conflict with some of the other libraries with the SPI interface
const uint8_t radioCSPin = 7;
const uint8_t radioResetPin = 32;
const uint8_t radioBusyPin = 31;
const uint8_t radioDIO1Pin = 30;
const uint8_t radioDIO2Pin = 27;
const float EBYTE_FREQ = 912.3;
SX1262 radio = new Module(radioCSPin, radioDIO1Pin, radioResetPin, radioBusyPin, SPI, spiSettings);
// SX1262 radio = new Module(radioCSPin, radioDIO1Pin, radioResetPin, radioBusyPin);

// Servo Pins
LSR_RollController rollCtrl;
constexpr uint8_t numberOfServos = 4;
const uint8_t ServoPins[numberOfServos] = {33, 36, 37, 14};
const uint16_t servoBitResolution = 12;
Servo servos[4]; // Change the servo library frequency from 20,000 to 4,000
const uint16_t minPulse = 1100;
const uint16_t maxPulse = 1900; 
const uint16_t neutralPulse = 1500; 

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

    // Use the IMU interrupts to trigger sensor readings
    attachInterrupt(digitalPinToInterrupt(sensors.getLSM6DSO32IntPin1()), changeIMUInterruptPin1, FALLING);
    attachInterrupt(digitalPinToInterrupt(sensors.getLSM6DSO32IntPin2()), changeIMUInterruptPin2, FALLING);

    /* Setup code here */
    // int16_t radioState = radio.begin(EBYTE_FREQ);
    // if(radioState != RADIOLIB_ERR_NONE) {
    //     Serial.printf("Failed to initialize radio, error code: %d\n", radioState);
    // } else {
    //     radioPresent = true;
    //     Serial.printf(F("EByte Initalized\n"));
    // }

    if(!sdCard.begin(SdioConfig(FIFO_SDIO))) {
        sdCard.initErrorPrint(&Serial);
    } else {
        SDcardPresent = true;

        if(!SDTimer.begin(SDWriteTimerCallback, SDWriteFreqMicroseconds / 10)) {
            Serial.printf(F("SD Timer Failed\n"));
        }

        if (!sdCard.exists("/logs")) {
            sdCard.mkdir("/logs");
        }

        if(sdCard.exists("/logs/log.csv")) {
            uint16_t fileNumber = 1;
            while(sdCard.exists(("/logs/log" + String(fileNumber) + ".csv").c_str())) {
                fileNumber++;
            }
            SDcardPresent = sdFile.open(("/logs/log" + String(fileNumber) + ".csv").c_str(), O_RDWR | O_CREAT | O_TRUNC);
        } else {
            SDcardPresent = sdFile.open("/logs/log.csv", O_RDWR | O_CREAT | O_TRUNC);
        }

        if(!SDcardPresent) {
            Serial.printf(F("Failed to open file on SD card\n"));
        }

        while(!sdFile) {
            delayMicroseconds(2000);
        }

        if(!sdFile.preAllocate(10 * 1024 * 1024)) { // Pre-allocate 10MB for better write speeds
            Serial.printf(F("Failed to pre-allocate SD card\n"));
        }
        sdFile.println("TimeStamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,VelX,VelY,VelZ,PosX,PosY,PosZ,Theta,Phi,Psi,Pressure,FlightState");
        sdFile.flush();
        sdBuffer.begin(&sdFile);
    }
    
    // Initalize the Servo Pins && PWM 
    analogWriteRes(servoBitResolution); // Set resolution of the analogwrite function
    
    for (uint8_t pin = 0; pin < numberOfServos; pin++) {
        pinMode(pin, OUTPUT);
        servos[pin].attach(ServoPins[pin]);
        servos[pin].writeMicroseconds(neutralPulse);
    } 

    accelAltTimer = millis();
    GPSTimer = millis();

    // DO NOT DELETE!!!!!!!!!!
    sensors.updateNoKalmanFilter(data);
    ring.push(data);
}

void loop() { 
    // Set of commands that can allow us to make state changes to test the program at will
    // TODO: Add some commands to manipulate the servo positions, Zero/Reset sensors
    #if __TEST__ 
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
                sdFile.flush();
            } else if(serialTestCommand.equalsIgnoreCase("ZEROSERVOS")) {
                for(uint8_t pins : ServoPins) {
                    servos[pins].writeMicroseconds(neutralPulse);
                }
            } else if(serialTestCommand.equalsIgnoreCase("SWEEPSERVOS")) {
                uint8_t count = 0;
                uint16_t angle = neutralPulse; 
                uint16_t increament = 4;
                const uint16_t servoSweepDelay = 5; 
                elapsedMillis servoSweepTimer = 0;
                while(count < 4) {
                    if(servoSweepTimer < servoSweepDelay) {
                        continue;
                    }
                     
                    servoSweepTimer = 0;
                    angle += increament;
                    if(angle > maxPulse || angle <= minPulse) {
                        increament = -increament;
                        count++;
                    }

                    for(uint8_t pins = 0; pins < numberOfServos; pins++) {
                        servos[pins].writeMicroseconds(angle);
                    }
                }

                for(uint8_t pins = 0; pins < numberOfServos; pins++) {
                    servos[pins].writeMicroseconds(neutralPulse);
                }
            } else if(serialTestCommand.equalsIgnoreCase("REBOOT")) {
                if(sdFile) {
                    sdFile.flush();
                    sdFile.close();
                }
                _reboot_Teensyduino_();
            } else {
                Serial.printf(F("Invalid Serial Command\n"));
            } 
        }
    #endif

    if (sensors.lsmDataReadyInt1 || sensors.lsmDataReadyInt2) { 
        sensors.lsmDataReadyInt1 = false;
        sensors.lsmDataReadyInt2 = false;

        // Calculate dt using microsecond precision
        static uint32_t lastMicros;
        uint32_t currentMicros = micros();
        if (lastMicros == 0) lastMicros = currentMicros;
        float current_dt = (float)(currentMicros - lastMicros) / 1000000.0f;
        if(current_dt <= 0) {
            current_dt = 0.01;
        }
        lastMicros = currentMicros;

        // READ SENSORS ONCE
        sensors.updateNoKalmanFilter(data);
        ring.push(data);

        switch (currentFlightState) {
            case PRE_LAUNCH: {
                if (launchDetect(ring)) {
                    Serial.printf(F("--Burn--"));
                    currentFlightState = BURN;
                    launchTime = millis();
                    /* code to log entire ring goes here */

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
                    sdFile.flush();
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
                    sdFile.flush();

                    for (uint8_t pin = 0; pin < numberOfServos; pin++) {
                        servos[pin].writeMicroseconds(neutralPulse);
                    }
                    
                    break;
                } else {
                    /* Coast code here */

                    // Calculate how long we've been in COAST
                    uint32_t elapsed = millis() - burnTime;
                    static bool isReturning;
                    float targetRoll = 90.0;
                    const uint16_t angleThreshold = 2;
                    const uint32_t returnTimeThreshold = 1000;

                    if(isReturning) {
                        targetRoll = 0.0;
                    } else {
                        targetRoll = 90.0;
                    }

                    float deflection = rollCtrl.update(data, targetRoll, isReturning, current_dt);               
                    int pulseValue = map(deflection, -MAX_FIN_ANGLE, MAX_FIN_ANGLE, 1500 + ((800 / 90.0) * -MAX_FIN_ANGLE), 1500 + ((800 / 90.0) * MAX_FIN_ANGLE));

                    if(data.Theta > targetRoll + angleThreshold || data.Theta < targetRoll - angleThreshold)  { 
                        for (uint8_t pin = 0; pin < numberOfServos; pin++) {
                            servos[pin].writeMicroseconds(pulseValue);
                        }

                    } else {

                        static elapsedMillis returnTimer;
                        if(returnTimer > returnTimeThreshold) {
                            isReturning = true;
                        }
                    }
                    
                    Serial.printf("Target Roll: %.2f\n, Theta: %.2f\n, Pulse: %d\n\n", targetRoll, data.Theta, pulseValue);

                    writePacketToSD(ring.getFirst());
                    break;
                }
            }
            case DESCENT: {
                if (landingDetect(ring)) {
                    Serial.printf(F("--Landed--"));
                    currentFlightState = LANDED;
                    landTime = millis();
                    sdFile.flush();
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
                    sdFile.flush();
                    sdFile.close();
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
        altitudeIncreasing = false;
    } else {
        altitudeIncreasing = true;
    }

    if((accelCount >= samplesRequired) && !altitudeIncreasing) {
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
    #if __TEST__
        return false;
    #endif

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

    // Flush the SD card every 2.5 seconds to prevent data loss
    const uint16_t SDFlushInterval = 2500; 

    if(!writeToSD) {
        interrupts();
        return;
    }

    if (!SDcardPresent) {
        interrupts();
        return;
    }

    if(sdFile.isBusy()) {
        interrupts();
        return;
    }

    if (sdFile) {

        sdBuffer.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n", 
            data.TimeStamp, 
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

        if(SDWriteElapsedTime > SDFlushInterval || sdBuffer.bytesFree() < 512) {
            sdBuffer.sync();
            sdFile.flush();
            SDWriteElapsedTime = 0;
        }
    } else {
        Serial.println(F("Failed to write to SD card!"));
    }

    writeToSD = false;
    interrupts();
}

void changeIMUInterruptPin1(void) {
    sensors.dataReadyLSMInt1();
}

void changeIMUInterruptPin2(void) {
    sensors.dataReadyLSMInt2();
}