#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

// ── High-Level Movement Primitives ───────────────────────
//
// All functions are BLOCKING — they return only when the
// movement is complete.  Each contains an internal loop that
// updates ToF sensors and runs the PID controllers.

void motion_init();             // Create PID instances — call once in setup()
void motion_forward(uint8_t cells);
void motion_forward_speed(uint8_t cells, float speed_target);  // speed-run variant
void motion_turn_left();
void motion_turn_right();
void motion_turn_around();
void motion_stop();
void motion_square_up();        // Align perpendicular to front wall

#endif // MOTION_H
