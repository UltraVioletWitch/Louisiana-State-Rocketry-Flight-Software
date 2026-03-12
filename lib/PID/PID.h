#pragma once 

#include <algorithm>
#include <Arduino.h>

class PID {
    public:
        // Constructors
        PID(double kp, double ki, double kd);
        PID(double kp, double ki, double kd, double dt);
        PID(double kp, double ki, double kd, double dt, double min_output, double max_output);

        // Methods
        double calculate(double setpoint, double measured_value);

        // Setters
        void reset();
        void setTunedParameters(double kp, double ki, double kd);
        void setPIDOutputLimits(double min_output, double max_output);

        // Getters
        double getKp() const;
        double getKi() const;
        double getKd() const;
        double getDt() const;
        double getMinOutput() const;
        double getMaxOutput() const;
        double getIntegral() const;
        double getDerivative() const; // This derivative term can cause issues if the PID is used in a loop with a very small dt
        double getPreviousError() const;
        double getPreviousTime() const;

    private:
        double _kp;
        double _ki;
        double _kd;
        double _dt;
        double _min_output;
        double _max_output;
        double _integral;
        double _derivative;
        double _previous_error;
        double _previous_time;
};