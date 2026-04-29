#include "tof.h"
#include "config.h"
#include <Wire.h>
#include <VL53L0X.h>

static VL53L0X sensorL;
static VL53L0X sensorF;
static VL53L0X sensorR;

static uint16_t dist_left  = TOF_OUT_OF_RANGE;
static uint16_t dist_front = TOF_OUT_OF_RANGE;
static uint16_t dist_right = TOF_OUT_OF_RANGE;

// ── Initialise all three VL53L0X sensors ─────────────────
// XSHUT sequence: all OFF → stagger ON → set unique addresses
bool tof_init() {
    // Configure XSHUT pins as outputs
    pinMode(PIN_TOF_XSHUT_L, OUTPUT);
    pinMode(PIN_TOF_XSHUT_F, OUTPUT);
    pinMode(PIN_TOF_XSHUT_R, OUTPUT);

    // Step 1: All sensors OFF
    digitalWrite(PIN_TOF_XSHUT_L, LOW);
    digitalWrite(PIN_TOF_XSHUT_F, LOW);
    digitalWrite(PIN_TOF_XSHUT_R, LOW);
    delay(50);

    // Step 2: Left sensor ON → assign address 0x30
    digitalWrite(PIN_TOF_XSHUT_L, HIGH);
    delay(50);
    sensorL.setTimeout(500);
    if (!sensorL.init()) {
        Serial.println(F("[TOF] ERROR: Left sensor init failed"));
        return false;
    }
    sensorL.setAddress(TOF_ADDR_LEFT);
    Serial.println(F("[TOF] Left  sensor → 0x30"));

    // Step 3: Front sensor ON → assign address 0x31
    digitalWrite(PIN_TOF_XSHUT_F, HIGH);
    delay(50);
    sensorF.setTimeout(500);
    if (!sensorF.init()) {
        Serial.println(F("[TOF] ERROR: Front sensor init failed"));
        return false;
    }
    sensorF.setAddress(TOF_ADDR_FRONT);
    Serial.println(F("[TOF] Front sensor → 0x31"));

    // Step 4: Right sensor ON → assign address 0x32
    digitalWrite(PIN_TOF_XSHUT_R, HIGH);
    delay(50);
    sensorR.setTimeout(500);
    if (!sensorR.init()) {
        Serial.println(F("[TOF] ERROR: Right sensor init failed"));
        return false;
    }
    sensorR.setAddress(TOF_ADDR_RIGHT);
    Serial.println(F("[TOF] Right sensor → 0x32"));

    // Configure timing budget and start continuous measurement
    sensorL.setMeasurementTimingBudget(TOF_TIMING_BUDGET);
    sensorF.setMeasurementTimingBudget(TOF_TIMING_BUDGET);
    sensorR.setMeasurementTimingBudget(TOF_TIMING_BUDGET);

    sensorL.startContinuous();
    sensorF.startContinuous();
    sensorR.startContinuous();

    Serial.println(F("[TOF] All sensors initialised — continuous mode"));
    return true;
}

void tof_update() {
    uint16_t raw;

    raw = sensorL.readRangeContinuousMillimeters();
    dist_left = (raw == 65535 || raw > TOF_OUT_OF_RANGE) ? TOF_OUT_OF_RANGE : raw;

    raw = sensorF.readRangeContinuousMillimeters();
    dist_front = (raw == 65535 || raw > TOF_OUT_OF_RANGE) ? TOF_OUT_OF_RANGE : raw;

    raw = sensorR.readRangeContinuousMillimeters();
    dist_right = (raw == 65535 || raw > TOF_OUT_OF_RANGE) ? TOF_OUT_OF_RANGE : raw;
}

uint16_t tof_get_left()  { return dist_left;  }
uint16_t tof_get_front() { return dist_front; }
uint16_t tof_get_right() { return dist_right; }

bool tof_wall_left()  { return dist_left  < TOF_WALL_PRESENT; }
bool tof_wall_front() { return dist_front < TOF_WALL_PRESENT; }
bool tof_wall_right() { return dist_right < TOF_WALL_PRESENT; }
