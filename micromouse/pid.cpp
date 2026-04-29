#include "pid.h"
#include <Arduino.h>   // for constrain()

PID::PID(float kp, float ki, float kd, float out_min, float out_max)
    : _kp(kp), _ki(ki), _kd(kd),
      _integral(0.0f), _last_error(0.0f),
      _out_min(out_min), _out_max(out_max),
      _first(true) {}

float PID::compute(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measured;

    // Proportional
    float p = _kp * error;

    // Integral with anti-windup (clamp)
    _integral += error * dt;
    float i = _ki * _integral;
    // Clamp integrator contribution to output limits
    if (i > _out_max) { i = _out_max; _integral = _out_max / _ki; }
    if (i < _out_min) { i = _out_min; _integral = _out_min / _ki; }

    // Derivative (skip on first call to avoid kick)
    float d = 0.0f;
    if (!_first) {
        d = _kd * (error - _last_error) / dt;
    }
    _first = false;
    _last_error = error;

    // Sum and clamp
    float output = p + i + d;
    return constrain(output, _out_min, _out_max);
}

void PID::reset() {
    _integral   = 0.0f;
    _last_error = 0.0f;
    _first      = true;
}
