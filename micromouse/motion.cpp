#include "motion.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "tof.h"
#include "pid.h"

// ── PID instances (file-scope) ───────────────────────────
static PID pid_speed_L(PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD, 0, PWM_MAX);
static PID pid_speed_R(PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD, 0, PWM_MAX);
static PID pid_steer(PID_STEER_KP, PID_STEER_KI, PID_STEER_KD, -50, 50);

void motion_init() {
    pid_speed_L.reset();
    pid_speed_R.reset();
    pid_steer.reset();
    Serial.println(F("[MOT] Motion PIDs initialised"));
}

// ── Steering error computation (single + dual wall) ──────
static float _compute_steering_error() {
    bool wl = tof_wall_left();
    bool wr = tof_wall_right();

    if (wl && wr) {
        // Both walls: centre between them
        return (float)tof_get_left() - (float)tof_get_right();
    } else if (wl) {
        // Left wall only: maintain SINGLE_WALL_TARGET_MM from it
        return (float)tof_get_left() - SINGLE_WALL_TARGET_MM;
    } else if (wr) {
        // Right wall only: maintain SINGLE_WALL_TARGET_MM from it
        return SINGLE_WALL_TARGET_MM - (float)tof_get_right();
    }
    // No walls — dead reckoning, no correction
    return 0.0f;
}

// ══════════════════════════════════════════════════════════
//  FORWARD
// ══════════════════════════════════════════════════════════
void motion_forward_speed(uint8_t cells, float speed_target) {
    encoder_reset();
    pid_speed_L.reset();
    pid_speed_R.reset();
    pid_steer.reset();

    float target_counts = cells * CELL_SIZE_MM / MM_PER_COUNT;
    float accel_zone    = (float)MOVE_ACCEL_COUNTS;
    float decel_start   = target_counts - (float)MOVE_DECEL_COUNTS;

    int64_t prev_l = 0, prev_r = 0;
    uint32_t last_ctrl   = millis();
    uint32_t last_sensor = millis();

    while (true) {
        uint32_t now = millis();

        // ── Sensor update (non-blocking) ─────────────────
        if (now - last_sensor >= SENSOR_LOOP_MS) {
            last_sensor = now;
            tof_update();
        }

        // ── Control tick ─────────────────────────────────
        if (now - last_ctrl >= CONTROL_LOOP_MS) {
            float dt = (now - last_ctrl) / 1000.0f;
            last_ctrl = now;

            int64_t enc_l = encoder_get_left();
            int64_t enc_r = encoder_get_right();
            float avg = ((float)abs(enc_l) + (float)abs(enc_r)) / 2.0f;

            // Done?
            if (avg >= target_counts) break;

            // Measure speed (counts per tick)
            float spd_l = (float)(enc_l - prev_l);
            float spd_r = (float)(enc_r - prev_r);
            prev_l = enc_l;
            prev_r = enc_r;

            // Ramp speed target (trapezoidal profile)
            float ramp = speed_target;
            if (avg < accel_zone) {
                ramp = speed_target * (0.3f + 0.7f * avg / accel_zone);
            } else if (avg > decel_start && decel_start > accel_zone) {
                float remaining = target_counts - avg;
                float decel_len = target_counts - decel_start;
                ramp = speed_target * (0.3f + 0.7f * remaining / decel_len);
            }

            // Speed PID per motor
            float pwm_l = pid_speed_L.compute(ramp, spd_l, dt);
            float pwm_r = pid_speed_R.compute(ramp, spd_r, dt);

            // Steering PID
            float steer_err = _compute_steering_error();
            float steer_out = pid_steer.compute(0.0f, steer_err, dt);

            motor_set((int)(pwm_l - steer_out),
                      (int)(pwm_r + steer_out));
        }
    }

    motion_stop();
}

void motion_forward(uint8_t cells) {
    motion_forward_speed(cells, TARGET_SPEED_COUNTS);
}

// ══════════════════════════════════════════════════════════
//  TURNS (in-place pivot)
// ══════════════════════════════════════════════════════════
//
// 90° turn arc per wheel = π × WHEEL_BASE_MM / 4
// Target counts = arc_mm / MM_PER_COUNT

static void _do_turn(bool clockwise) {
    encoder_reset();
    pid_speed_L.reset();
    pid_speed_R.reset();

    float arc_mm = PI * WHEEL_BASE_MM / 4.0f;
    float target = arc_mm / MM_PER_COUNT;

    int64_t prev_l = 0, prev_r = 0;
    uint32_t last_ctrl = millis();

    while (true) {
        uint32_t now = millis();
        if (now - last_ctrl >= CONTROL_LOOP_MS) {
            float dt = (now - last_ctrl) / 1000.0f;
            last_ctrl = now;

            int64_t enc_l = encoder_get_left();
            int64_t enc_r = encoder_get_right();
            float avg = ((float)abs(enc_l) + (float)abs(enc_r)) / 2.0f;

            if (avg >= target) break;

            float spd_l = (float)(abs(enc_l) - abs(prev_l));
            float spd_r = (float)(abs(enc_r) - abs(prev_r));
            prev_l = enc_l;
            prev_r = enc_r;

            // Constant-speed PID (no steering during turns)
            float turn_speed = (float)MOTOR_TURN_SPEED * TARGET_SPEED_COUNTS / (float)MOTOR_BASE_SPEED;
            float pwm_l = pid_speed_L.compute(turn_speed, spd_l, dt);
            float pwm_r = pid_speed_R.compute(turn_speed, spd_r, dt);

            if (clockwise) {
                // Right turn: left forward, right reverse
                motor_set((int)pwm_l, -(int)pwm_r);
            } else {
                // Left turn: left reverse, right forward
                motor_set(-(int)pwm_l, (int)pwm_r);
            }
        }
    }

    motor_brake();
    delay(TURN_SETTLE_MS);
    encoder_reset();
    pid_speed_L.reset();
    pid_speed_R.reset();
}

void motion_turn_left()  { _do_turn(false); }
void motion_turn_right() { _do_turn(true);  }

void motion_turn_around() {
    // 180° = two 90° turns in the same direction
    _do_turn(true);
    _do_turn(true);
}

// ══════════════════════════════════════════════════════════
//  STOP
// ══════════════════════════════════════════════════════════
void motion_stop() {
    motor_brake();
    pid_speed_L.reset();
    pid_speed_R.reset();
    pid_steer.reset();
    encoder_reset();
}

// ══════════════════════════════════════════════════════════
//  SQUARE-UP  (align perpendicular to front wall)
// ══════════════════════════════════════════════════════════
//
// Strategy with a single front sensor:
//   1. Scan left & right by small rotations, recording front distance
//      at each step.
//   2. The angle giving the MINIMUM distance is perpendicular.
//   3. Rotate back to that angle.
//   4. Creep forward/backward to reach SQUARE_UP_TARGET_MM.
//
// This works because the shortest distance through the sensor
// cone is the perpendicular ray to the wall surface.

void motion_square_up() {
    tof_update();
    uint16_t fwd = tof_get_front();

    if (fwd > SQUARE_UP_THRESHOLD_MM || fwd == TOF_OUT_OF_RANGE) return;

    // ── Phase 1: Rotational scan to find perpendicular ───
    //
    // We'll nudge the robot in small increments left and right,
    // measuring front distance.  The minimum reading indicates
    // perpendicular alignment.

    // Record where we are now
    encoder_reset();
    uint16_t best_dist  = fwd;
    int      best_step  = 0;   // 0 = current position

    // Small rotation per step (in encoder counts)
    // ~1° ≈ (π × 92 / 360) / 0.0576 ≈ 14 counts per degree
    const int counts_per_step = 14;   // ~1° rotation

    // Scan LEFT (negative steps)
    for (int s = 1; s <= SQUARE_UP_SCAN_STEPS; s++) {
        encoder_reset();
        // Rotate left by one step
        uint32_t t0 = millis();
        while ((float)(abs(encoder_get_left()) + abs(encoder_get_right())) / 2.0f < counts_per_step) {
            motor_set(-(int)SQUARE_UP_SCAN_PWM, (int)SQUARE_UP_SCAN_PWM);
            if (millis() - t0 > 500) break;  // timeout safety
        }
        motor_brake();
        delay(30);

        tof_update();
        uint16_t d = tof_get_front();
        if (d < best_dist) {
            best_dist = d;
            best_step = -s;
        }
    }

    // Return to centre from left scan
    for (int s = 0; s < SQUARE_UP_SCAN_STEPS; s++) {
        encoder_reset();
        uint32_t t0 = millis();
        while ((float)(abs(encoder_get_left()) + abs(encoder_get_right())) / 2.0f < counts_per_step) {
            motor_set((int)SQUARE_UP_SCAN_PWM, -(int)SQUARE_UP_SCAN_PWM);
            if (millis() - t0 > 500) break;
        }
        motor_brake();
        delay(20);
    }

    // Scan RIGHT (positive steps)
    for (int s = 1; s <= SQUARE_UP_SCAN_STEPS; s++) {
        encoder_reset();
        uint32_t t0 = millis();
        while ((float)(abs(encoder_get_left()) + abs(encoder_get_right())) / 2.0f < counts_per_step) {
            motor_set((int)SQUARE_UP_SCAN_PWM, -(int)SQUARE_UP_SCAN_PWM);
            if (millis() - t0 > 500) break;
        }
        motor_brake();
        delay(30);

        tof_update();
        uint16_t d = tof_get_front();
        if (d < best_dist) {
            best_dist = d;
            best_step = s;
        }
    }

    // Return to centre from right scan
    for (int s = 0; s < SQUARE_UP_SCAN_STEPS; s++) {
        encoder_reset();
        uint32_t t0 = millis();
        while ((float)(abs(encoder_get_left()) + abs(encoder_get_right())) / 2.0f < counts_per_step) {
            motor_set(-(int)SQUARE_UP_SCAN_PWM, (int)SQUARE_UP_SCAN_PWM);
            if (millis() - t0 > 500) break;
        }
        motor_brake();
        delay(20);
    }

    // ── Phase 2: Rotate to best angle ────────────────────
    if (best_step != 0) {
        int steps_to_go = abs(best_step);
        bool go_right = (best_step > 0);

        for (int s = 0; s < steps_to_go; s++) {
            encoder_reset();
            uint32_t t0 = millis();
            while ((float)(abs(encoder_get_left()) + abs(encoder_get_right())) / 2.0f < counts_per_step) {
                if (go_right)
                    motor_set((int)SQUARE_UP_SCAN_PWM, -(int)SQUARE_UP_SCAN_PWM);
                else
                    motor_set(-(int)SQUARE_UP_SCAN_PWM, (int)SQUARE_UP_SCAN_PWM);
                if (millis() - t0 > 500) break;
            }
            motor_brake();
            delay(20);
        }
    }

    // ── Phase 3: Creep to target distance ────────────────
    tof_update();
    fwd = tof_get_front();
    uint32_t creep_start = millis();

    while (abs((int)fwd - (int)SQUARE_UP_TARGET_MM) > SQUARE_UP_TOLERANCE_MM) {
        if (millis() - creep_start > 2000) break;  // safety timeout

        if (fwd > SQUARE_UP_TARGET_MM) {
            motor_set(SQUARE_UP_SPEED, SQUARE_UP_SPEED);   // creep forward
        } else {
            motor_set(-SQUARE_UP_SPEED, -SQUARE_UP_SPEED); // creep back
        }
        delay(CONTROL_LOOP_MS);
        tof_update();
        fwd = tof_get_front();
    }

    motor_brake();
    encoder_reset();

    Serial.print(F("[MOT] Square-up done. Front dist = "));
    Serial.println(fwd);
}
