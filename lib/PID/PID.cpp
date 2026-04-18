#include "PID.h"

LSR_RollController::LSR_RollController() {
    resetController();
}

void LSR_RollController::resetController() {
    innerLoop.reset();
    outerLoop.reset();
    thetaFilter.clear();
    gyroFilter.clear();
}

/*
     Line Logic: Intelligent Clamping Anti-Windup
     Only accumulates integral if the output isn't already saturated
*/
float LSR_PID_Core::compute(float target, float current, float kp, float ki, float kd, float dt, float limit) {
    // Safety check for dt very unlikly
    if (dt < MIN_DT) 
        return 0.0; 

    float error = target - current;
    //Proportional term
    float P = kp * error;
    // Derivative on Measurement 
    // Formula: D = -Kd * (d_current / dt)
    float D = -kd * (current - prevMeasurement) / dt;
    prevError = error;
    prevMeasurement = current;

    // Only integrate if not saturated
    float potentialOutput = P + (ki * (integral + error * dt)) + D;
    if (abs(potentialOutput) < limit) {
        integral += error * dt;
    }

    float I = ki * integral;
    return constrain(P + I + D, -limit, limit);
}

float LSR_RollController::update(E22_Packet &packet, float targetRoll, bool isReturning, float dt) {
    thetaFilter.add(packet.Theta);
    gyroFilter.add(packet.GyroX);

    float smoothTheta = thetaFilter.getAvg();
    float smoothGyro = gyroFilter.getAvg();

    //  TARGET RAMPING LOGIC 
    // Calculate how much we can move setpoint to avoid PID kick
    float maxChange = RAMP_RATE * dt;
    float setpointDiff = targetRoll - currentSetpoint;

    // Move currentSetpoint toward targetRoll by no more than maxChange
    if (abs(setpointDiff) <= maxChange) {
        currentSetpoint = targetRoll;
    } else {
        currentSetpoint += (setpointDiff > 0 ? maxChange : -maxChange);
    }

    // Select Gain Set
    float kpo, kio, kdo, kpi, kii, kdi;
    if (!isReturning) {
        kpo = Kp_INIT_OUTER; kio = Ki_INIT_OUTER; kdo = Kd_INIT_OUTER;
        kpi = Kp_INIT_INNER; kii = Ki_INIT_INNER; kdi = Kd_INIT_INNER;
    } else {
        kpo = Kp_RET_OUTER; kio = Ki_RET_OUTER; kdo = Kd_RET_OUTER;
        kpi = Kp_RET_INNER; kii = Ki_RET_INNER; kdi = Kd_RET_INNER;
    }

    // Outer Loop: Angle -> Target Rate (Uses the RAMPED setpoint)
    float targetRate = outerLoop.compute(currentSetpoint, smoothTheta, kpo, kio, kdo, dt, MAX_ROLL_RATE);

    // Inner Loop: Rate -> Fin Angle
    float finAngle = innerLoop.compute(targetRate, smoothGyro, kpi, kii, kdi, dt, MAX_FIN_ANGLE);

    return finAngle;
}