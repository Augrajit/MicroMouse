#ifndef SOLVER_H
#define SOLVER_H

#include <Arduino.h>
#include "led.h"    // RobotState is typedef'd here

// ── Heading (which direction the robot physically faces) ──
typedef enum { HEADING_N = 0, HEADING_E = 1, HEADING_S = 2, HEADING_W = 3 } Heading;

void       solver_init();
void       solver_step();            // Call every loop() tick
RobotState solver_get_state();

// Expose position for debug
uint8_t solver_get_row();
uint8_t solver_get_col();
Heading solver_get_heading();

#endif // SOLVER_H
