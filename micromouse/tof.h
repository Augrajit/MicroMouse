#ifndef TOF_H
#define TOF_H

#include <Arduino.h>

// ── VL53L0X Time-of-Flight Sensor API ────────────────────
//
// Manages 3× VL53L0X sensors on a shared I2C bus.
// XSHUT pins are used to stagger power-on and assign unique
// I2C addresses at startup.
//
// Library: VL53L0X by Pololu (vl53l0x-arduino)

bool     tof_init();        // Returns false if any sensor fails
void     tof_update();      // Read all 3 sensors (call every SENSOR_LOOP_MS)

uint16_t tof_get_left();    // Last reading in mm
uint16_t tof_get_front();
uint16_t tof_get_right();

bool     tof_wall_left();   // true if reading < TOF_WALL_PRESENT
bool     tof_wall_front();
bool     tof_wall_right();

#endif // TOF_H
