#include "PID.h"

PID::PID(double kp, double ki, double kd) 
    : PID(kp, ki, kd, 1, __DBL_MIN__, __DBL_MAX__) {}

PID::PID(double kp, double ki, double kd, double dt)
    : PID(kp, ki, kd, dt, __DBL_MIN__, __DBL_MAX__) {}

PID::PID(double kp, double ki, double kd, double dt, double min_output, double max_output)
    : _kp(kp), _ki(ki), _kd(kd), _dt(dt), _min_output(min_output), _max_output(max_output), _integral(0), _derivative(0), _previous_error(0), _previous_time(0) {}

double PID::calculate(double setpoint, double measured_value) {
    unsigned int timeNow = millis();
    _dt = (timeNow - _previous_time) / 1000.0;
    if(_dt <= 0) {
        _dt = 1;
    }

    double error = setpoint - measured_value;
    _integral += error * _dt;
    _derivative = (error - _previous_error) / _dt;
    

    _previous_error = error;
    _previous_time = timeNow;

    double output = _kp * error + _ki * _integral + _kd * _derivative;
    
    output = std::clamp(output, _min_output, _max_output);

    return output;
}

void PID::reset() {
    _kp = 1.0;
    _ki = 1.0;
    _kd = 1.0;
    _dt = 1.0;
    _min_output = __DBL_MIN__;
    _max_output = __DBL_MAX__;
    _integral = 0;
    _derivative = 0;
    _previous_error = 0;
    _previous_time = 0;
}

void PID::setTunedParameters(double kp, double ki, double kd) {
    if(kp <= 0) {
        Serial.println(F("Invalid Kp value. It must be greater than 0."));
        return;
    }

    if(ki < 0) {
        Serial.println(F("Invalid Ki value. It must be greater than or equal to 0."));
        return;
    }

    if(kd < 0) {
        Serial.println(F("Invalid Kd value. It must be greater than or equal to 0."));
        return;
    }

    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setPIDOutputLimits(double min_output, double max_output) {
    if(min_output >= max_output) {
        return;
    }

    _min_output = min_output;
    _max_output = max_output;
}

double PID::getKp() const {
    return _kp;
}

double PID::getKi() const {
    return _ki;
}

double PID::getKd() const {
    return _kd;
}

double PID::getDt() const {
    return _dt;
}

double PID::getMinOutput() const {
    return _min_output;
}

double PID::getMaxOutput() const {
    return _max_output;
}

double PID::getIntegral() const {
    return _integral;
}

double PID::getDerivative() const {
    return _derivative;
}

double PID::getPreviousError() const {
    return _previous_error;
}

double PID::getPreviousTime() const {
    return _previous_time;
}