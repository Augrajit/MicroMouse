#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// ── Quadrature Encoder API (ESP32Encoder / PCNT) ─────────
//
// Uses the ESP32 hardware Pulse Counter (PCNT) peripheral
// via the ESP32Encoder library.  Full quadrature decode is
// done in silicon — zero CPU load, no missed pulses.
//
// Library: ESP32Encoder by Kevin Harrington
//          Install from Arduino Library Manager.

void    encoder_init();
int64_t encoder_get_left();
int64_t encoder_get_right();
void    encoder_reset();
float   encoder_to_mm(int64_t counts);

#endif // ENCODER_H
