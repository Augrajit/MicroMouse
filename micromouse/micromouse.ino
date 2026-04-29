// ══════════════════════════════════════════════════════════
//  MicroMouse — Main Sketch
// ══════════════════════════════════════════════════════════
//
//  Board:  ESP32-S3 Dev Module (16 MB Flash, OPI PSRAM)
//  Libs:   VL53L0X (Pololu), Adafruit NeoPixel, ESP32Encoder
//
//  Startup sequence:
//    1. Initialise Serial, LED, Motors, Encoders, I2C, ToF
//    2. Wait for button press  → EXPLORE
//       or Serial 'C'         → CALIBRATE
//
//  Loop:
//    • ToF reads every SENSOR_LOOP_MS (non-blocking)
//    • Solver state machine each tick
//    • LED animation each tick

#include <Wire.h>
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "tof.h"
#include "pid.h"
#include "motion.h"
#include "maze.h"
#include "solver.h"
#include "led.h"

// ── Timing ───────────────────────────────────────────────
static uint32_t last_sensor_ms = 0;

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);                // allow USB-CDC to connect
    Serial.println();
    Serial.println(F("╔══════════════════════════════════╗"));
    Serial.println(F("║       MicroMouse  v1.0           ║"));
    Serial.println(F("║  ESP32-S3 · Flood Fill Solver    ║"));
    Serial.println(F("╚══════════════════════════════════╝"));

    // ── LED ──────────────────────────────────────────────
    led_init();
    led_set(40, 40, 40);    // dim white during boot

    // ── Motors ───────────────────────────────────────────
    motor_init();
    Serial.println(F("[BOOT] Motors OK"));

    // ── Encoders ─────────────────────────────────────────
    encoder_init();
    Serial.println(F("[BOOT] Encoders OK"));

    // ── I2C ──────────────────────────────────────────────
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);   // 400 kHz fast-mode I2C
    Serial.println(F("[BOOT] I2C OK"));

    // ── ToF Sensors ──────────────────────────────────────
    if (!tof_init()) {
        Serial.println(F("[BOOT] *** ToF INIT FAILED — check wiring ***"));
        // Blink red to indicate error
        while (true) {
            led_set(255, 0, 0);
            delay(300);
            led_set(0, 0, 0);
            delay(300);
        }
    }
    Serial.println(F("[BOOT] ToF OK"));

    // ── Button ───────────────────────────────────────────
    pinMode(PIN_BTN_START, INPUT_PULLUP);

    // ── Solver (maze + motion PIDs) ──────────────────────
    solver_init();

    Serial.println(F("[BOOT] ✓ Ready. Press BOOT to start or send 'C' for calibration."));
    led_set(0, 0, 0);
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    // ── Periodic sensor read (outside motion functions) ──
    // During IDLE / CALIBRATE the motion functions aren't running,
    // so we keep sensor data fresh here.
    if (now - last_sensor_ms >= SENSOR_LOOP_MS) {
        last_sensor_ms = now;
        tof_update();
    }

    // ── Solver state machine ─────────────────────────────
    solver_step();

    // ── LED animation ────────────────────────────────────
    led_update(solver_get_state());
}
