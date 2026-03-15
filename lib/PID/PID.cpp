#include "PID.h"

PID::PID(double kp = 1.0, 
    double ki = 0.0, 
    double kd = 0.1, 
    double dt = 0.1, 
    double min_output = __DBL_MIN__, 
    double max_output = __DBL_MAX__)
    : _kp(kp), 
    _ki(ki), 
    _kd(kd), 
    _dt(dt), 
    _min_output(min_output), 
    _max_output(max_output), 
    _integral(0.0), 
    _derivative(0.0), 
    _previous_error(0.0), 
    _previous_time(0.0) {
        // dt being zero is a big no
        if(_dt <= 0) {
            _dt = 0.1;
        }

        if(_kp <= 0 ) {
            _kp = 1.0;
        }

        if(_ki < 0) {
            _ki = 0.0;
        }

        if(_kd < 0) {
            _kd = 0.1;
        }

        if(_min_output >= _max_output) {
            _min_output = __DBL_MIN__;
            _max_output = __DBL_MAX__;
        }
    }

double PID::calculate(double setpoint = 0, double measured_value = 0, double dt_s = 0.1, bool filterDerivative = false) {
    unsigned int timeNow = micros();

    // No dividing by zero or negative time intervals
    if(dt_s > 0) {
        _dt = dt_s;
    } else {
        _dt = (timeNow - (unsigned long)_previous_time) / 1e6;
    }

    // If dt is still zero or negative, set it to a default value
    if(_dt <= 0) {
        _dt = 0.1;
    }

    double error = setpoint - measured_value;

    _integral += error * _dt;
    if(_ki != 0) {
        double _integrand_min = _min_output / _ki;
        double _integrand_max = _max_output / _ki;
        _integral = std::clamp(_integral, _integrand_min, _integrand_max); // or constrain
    }
    
    if(filterDerivative) {
        double raw_derivative = (error - _previous_error) / _dt;
        constexpr double alpha = 0.75; // not a good solution to make this a constantexpr
        _derivative = alpha * _previous_derivative + (1 - alpha) * raw_derivative;
        _previous_derivative = raw_derivative;
    } else {
        _derivative = (error - _previous_error) / _dt;
        _previous_derivative = _derivative;
    }
    
    _previous_error = error;
    _previous_time = timeNow;

    double output = _kp * error + _ki * _integral + _kd * _derivative;

    return std::clamp(output, _min_output, _max_output); // or constrain
}

void PID::reset() {
    _integral = 0.0;
    _previous_error = 0.0;
}

void PID::setTunedParameters(double kp, double ki, double kd) {
    if(kp <= 0) {
        Serial.printf(F("Invalid Kp value. It must be greater than 0."));
        return;
    }

    if(ki < 0) {
        Serial.printf(F("Invalid Ki value. It must be greater than or equal to 0."));
        return;
    }

    if(kd < 0) {
        Serial.printf(F("Invalid Kd value. It must be greater than or equal to 0."));
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

double PID::getPreviousDerivative() const {
    return _previous_derivative;
}

double PID::getPreviousError() const {
    return _previous_error;
}

double PID::getPreviousTime() const {
    return _previous_time;
}