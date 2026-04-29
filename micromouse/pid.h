#ifndef PID_H
#define PID_H

// ── Generic PID Controller ───────────────────────────────
//
// Reusable class with configurable gains and output clamping.
// Includes integrator anti-windup (clamp).

class PID {
public:
    PID(float kp, float ki, float kd, float out_min, float out_max);

    // Compute one PID step.  dt is in seconds.
    float compute(float setpoint, float measured, float dt);

    // Clear integrator and last-error (call before each new move).
    void reset();

private:
    float _kp, _ki, _kd;
    float _integral;
    float _last_error;
    float _out_min, _out_max;
    bool  _first;           // suppress derivative kick on first call
};

#endif // PID_H
