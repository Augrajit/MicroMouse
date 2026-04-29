#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// ── TB6612FNG Motor Driver API ───────────────────────────
//
// motor_set() accepts signed PWM: -255 (full reverse) … +255 (full forward).
// 0 = active brake (both IN pins LOW via TB6612 short-brake mode).

void motor_init();
void motor_set(int pwm_left, int pwm_right);
void motor_brake();
void motor_coast();

#endif // MOTOR_H
