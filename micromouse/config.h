#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ══════════════════════════════════════════════════════════
//  MicroMouse — Master Configuration
//  ALL tunable constants live here. Do NOT scatter magic
//  numbers anywhere else in the codebase.
// ══════════════════════════════════════════════════════════

// ── Motor Driver (TB6612FNG) ─────────────────────────────
#define PIN_MOTOR_L_PWM     1    // PWMA  — Left motor speed  (LEDC)
#define PIN_MOTOR_L_IN1     15   // AIN1  — Left motor direction  (swapped to fix direction)
#define PIN_MOTOR_L_IN2     2    // AIN2  — Left motor direction  (swapped to fix direction)
#define PIN_MOTOR_R_PWM     4    // PWMB  — Right motor speed (LEDC)
#define PIN_MOTOR_R_IN1     6    // BIN1  — Right motor direction  (swapped to fix direction)
#define PIN_MOTOR_R_IN2     5    // BIN2  — Right motor direction  (swapped to fix direction)
#define PIN_MOTOR_STBY      7    // STBY  — HIGH = active, LOW = coast/stop

// ── Quadrature Encoders ──────────────────────────────────
#define PIN_ENC_L_A         8    // Left encoder channel A
#define PIN_ENC_L_B         9    // Left encoder channel B
#define PIN_ENC_R_A         11   // Right encoder channel A  (swapped to fix sign)
#define PIN_ENC_R_B         10   // Right encoder channel B  (swapped to fix sign)

// ── VL53L0X XSHUT (active LOW) ──────────────────────────
#define PIN_TOF_XSHUT_L     12   // Left sensor shutdown
#define PIN_TOF_XSHUT_F     13   // Front sensor shutdown
#define PIN_TOF_XSHUT_R     21   // Right sensor shutdown

// ── I2C Bus (shared by all 3 ToF sensors) ───────────────
#define PIN_I2C_SDA         17
#define PIN_I2C_SCL         18

// ── User Interface ───────────────────────────────────────
#define PIN_BTN_START       0    // Boot button — active LOW, INPUT_PULLUP
#define PIN_LED             48   // Onboard NeoPixel

// ── Button Timing (ms) ──────────────────────────────────
#define BTN_DEBOUNCE_MS     50   // Ignore bounces shorter than this
#define BTN_LONG_PRESS_MS   2000 // Hold > 2s = long-press
#define BTN_DOUBLE_GAP_MS   400  // Two presses within 400ms = double-press
#define CAL_NUM_TESTS       5    // Number of calibration test slots

// ── VL53L0X I2C Addresses ───────────────────────────────
#define TOF_ADDR_LEFT       0x30
#define TOF_ADDR_FRONT      0x31
#define TOF_ADDR_RIGHT      0x32

// ── Maze ────────────────────────────────────────────────
#define MAZE_SIZE           16          // 16×16 cells
#define CELL_SIZE_MM        180.0f      // Standard IEEE cell (mm)
#define GOAL_ROW_MIN        7           // Goal zone rows 7–8 (0-indexed)
#define GOAL_ROW_MAX        8
#define GOAL_COL_MIN        7           // Goal zone cols 7–8
#define GOAL_COL_MAX        8

// ── Robot Physical Dimensions ────────────────────────────
#define WHEEL_DIAMETER_MM   44.0f       // MEASURED
#define WHEEL_BASE_MM       92.0f       // Center-to-center of wheels — MEASURED
#define GEAR_RATIO          200.0f      // Motor gear ratio — CONFIRMED

// ── Encoder ──────────────────────────────────────────────
#define ENCODER_PPR         3           // Pulses per revolution (motor shaft)
#define ENCODER_QUADRATURE  4           // 4x quadrature decoding
// Derived — do not change these
#define COUNTS_PER_MOTOR_REV   (ENCODER_PPR * ENCODER_QUADRATURE)             // = 12
#define COUNTS_PER_WHEEL_REV   ((float)COUNTS_PER_MOTOR_REV * GEAR_RATIO)     // = 2400
#define MM_PER_COUNT           ((float)(PI * WHEEL_DIAMETER_MM / COUNTS_PER_WHEEL_REV))  // ≈ 0.0576 mm

// ── Motor PWM ────────────────────────────────────────────
#define PWM_FREQ_HZ         20000       // 20 kHz — above audible range
#define PWM_RESOLUTION_BITS 8           // 0–255
#define PWM_MAX             255
#define MOTOR_BASE_SPEED    150         // Default cruise PWM (0–255)
#define MOTOR_TURN_SPEED    120         // PWM used during turns
#define SPEEDRUN_SPEED      210         // Speed-run PWM (~1.4× base)

// ── Target Speed ─────────────────────────────────────────
// Used by the speed PID as its setpoint (counts per control tick).
// Derived from a desired travel speed of ~150 mm/s.
//   counts/s  = 150 / MM_PER_COUNT ≈ 2604
//   counts/tick = 2604 × 0.005 ≈ 13
#define TARGET_SPEED_COUNTS  13.0f      // Counts per CONTROL_LOOP_MS tick — TUNE
#define SPEEDRUN_SPEED_COUNTS 18.0f     // Faster for speed run

// ── ToF Sensor Thresholds (mm) ───────────────────────────
#define TOF_WALL_PRESENT    100         // Wall IS present if reading < this
#define TOF_NO_WALL         130         // No wall if reading > this
#define TOF_MAX_RANGE       1200        // VL53L0X max reliable range
#define TOF_OUT_OF_RANGE    1000        // Treat readings above this as "no wall"
#define TOF_TIMING_BUDGET   33000       // Measurement timing budget (µs)

// ── Single-Wall Steering ─────────────────────────────────
#define SINGLE_WALL_TARGET_MM  90.0f    // Half cell width — target dist from one wall

// ── Square-Up ────────────────────────────────────────────
#define SQUARE_UP_THRESHOLD_MM  120     // Engage square-up when front wall < this
#define SQUARE_UP_TARGET_MM     50      // Target front distance after square-up
#define SQUARE_UP_SPEED         80      // Slow PWM for square-up creep
#define SQUARE_UP_SCAN_PWM      60      // PWM for rotational scan
#define SQUARE_UP_SCAN_STEPS    5       // Number of scan steps each direction
#define SQUARE_UP_TOLERANCE_MM  3       // Perpendicularity tolerance (mm)

// ── PID — Speed Controller (per motor) ──────────────────
#define PID_SPEED_KP        2.0f
#define PID_SPEED_KI        0.5f
#define PID_SPEED_KD        0.1f

// ── PID — Wall Centering (steering correction) ──────────
#define PID_STEER_KP        0.3f
#define PID_STEER_KI        0.0f
#define PID_STEER_KD        0.05f

// ── Motion ───────────────────────────────────────────────
#define MOVE_ACCEL_COUNTS   350         // ~20 mm ramp up  (350 × 0.0576 ≈ 20 mm)
#define MOVE_DECEL_COUNTS   350         // ~20 mm ramp down
#define TURN_SETTLE_MS      100         // Wait after turn before next action (ms)

// ── Loop Timing ──────────────────────────────────────────
#define CONTROL_LOOP_MS     5           // PID update interval — 200 Hz
#define SENSOR_LOOP_MS      20          // ToF read interval   — 50 Hz

// ── Wall Bitmask Directions ──────────────────────────────
#define WALL_N  0x01
#define WALL_E  0x02
#define WALL_S  0x04
#define WALL_W  0x08

#endif // CONFIG_H
