#ifndef LED_H
#define LED_H

#include <Arduino.h>

// ── Forward-declare RobotState so led.h compiles standalone ──
// (actual enum is in solver.h; we re-declare it here to avoid
//  circular includes)
typedef enum {
    STATE_IDLE = 0,
    STATE_CALIBRATE,
    STATE_EXPLORE,
    STATE_RETURN,
    STATE_SPEEDRUN,
    STATE_DONE
} RobotState;

void led_init();
void led_set(uint8_t r, uint8_t g, uint8_t b);
void led_update(RobotState state);   // call every loop() — handles animations

#endif // LED_H
