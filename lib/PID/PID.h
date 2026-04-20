#ifndef PID
#define PID

#include <Arduino.h>
#include "LSR_Struct.h"

extern LSR_Struct data;

// --- TUNING CONSTANTS (The 12 PID Terms) ---

// INITIAL (Launch to 90° Hold)
const float Kp_INIT_OUTER = 5.0, Ki_INIT_OUTER = 0.015, Kd_INIT_OUTER = 0.1;
const float Kp_INIT_INNER = 3.5, Ki_INIT_INNER = 0.01, Kd_INIT_INNER = 0.02;

// RETURN (Back to 0°)
const float Kp_RET_OUTER = 10.0, Ki_RET_OUTER = 0.1, Kd_RET_OUTER = 0.3;
const float Kp_RET_INNER = 3.0, Ki_RET_INNER = 0.01, Kd_RET_INNER = 0.2;

// PHYSICAL LIMITS
const float MAX_FIN_ANGLE = 12.0; // Degrees (Mechanical limit)
const float MAX_ROLL_RATE = 360.0; // Degrees/Sec (Aerodynamic limit)

// Calculation Constants
const float RAMP_RATE = 180.0; // ramp to swtich target angle
const float MIN_DT = 0.0001;   // min dt set very low for 100hz
// --------------------------------------------

class LSR_RingBuffer {
public:
    float values[RING_SIZE];
    uint16_t index = 0;

    void add(float val) { 
        values[index] = val; 
        index = (index + 1) % RING_SIZE; 
    }

    float getAvg() {
        float sum = 0;
        for(int i = 0; i < RING_SIZE; i++) sum += values[i];
        return sum / RING_SIZE;
    }

    void clear() { 
        for(int i = 0; i < RING_SIZE; i++) values[i] = 0; 
    }
};

class LSR_PID_Core {
public:
    float integral, prevError, prevMeasurement;
    void reset() { integral = 0; prevError = 0; prevMeasurement = 0; }
    float compute(float target, float current, float kp, float ki, float kd, float dt, float limit);
};

class LSR_RollController {
public:
    LSR_RollController();
    void resetController();

    /**
    * @brief Computes the required fin deflection in degrees.
    * @param targetRoll Target angle in degrees (e.g. 90.0 or 0.0)
    * @param isReturning False for Phase 1, True for Phase 2
    * @param dt Delta time in seconds (e.g. 0.01 for 100Hz)
    */
    float update(E22_Packet &packet, float targetRoll, bool isReturning, float dt);

private:
    LSR_PID_Core innerLoop; // Rate Controller
    LSR_PID_Core outerLoop; // Angle Controller
    LSR_RingBuffer thetaFilter;
    LSR_RingBuffer gyroFilter;

    float currentSetpoint = 0.0; // For ramping logic tracking
};

#endif