#include "solver.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "tof.h"
#include "motor.h"
#include "encoder.h"
#include "led.h"

// ══════════════════════════════════════════════════════════
//  Robot State
// ══════════════════════════════════════════════════════════
static RobotState state    = STATE_IDLE;
static uint8_t    robot_row = 0;
static uint8_t    robot_col = 0;
static Heading    robot_heading = HEADING_N;

// ══════════════════════════════════════════════════════════
//  Sensor-to-absolute-direction lookup
// ══════════════════════════════════════════════════════════
//
// The three body-fixed sensors map to absolute directions
// depending on the robot's current heading.
//
//  Heading  |  Left sensor  |  Front sensor  |  Right sensor
//  ─────────┼───────────────┼────────────────┼──────────────
//   North   |   West        |   North        |   East
//   East    |   North       |   East         |   South
//   South   |   East        |   South        |   West
//   West    |   South       |   West         |   North

static uint8_t _abs_dir_left(Heading h) {
    // Left sensor: heading rotated –90°  → (heading + 3) % 4
    return (h + 3) & 0x03;
}

static uint8_t _abs_dir_front(Heading h) {
    // Front sensor: same as heading
    return (uint8_t)h;
}

static uint8_t _abs_dir_right(Heading h) {
    // Right sensor: heading rotated +90° → (heading + 1) % 4
    return (h + 1) & 0x03;
}

// ══════════════════════════════════════════════════════════
//  Helpers
// ══════════════════════════════════════════════════════════

static bool _in_goal() {
    return (robot_row >= GOAL_ROW_MIN && robot_row <= GOAL_ROW_MAX &&
            robot_col >= GOAL_COL_MIN && robot_col <= GOAL_COL_MAX);
}

static bool _at_start() {
    return (robot_row == 0 && robot_col == 0);
}

// Update walls from current ToF readings at current position + heading
static void _read_and_set_walls() {
    tof_update();

    maze_set_wall(robot_row, robot_col,
                  _abs_dir_left(robot_heading), tof_wall_left());
    maze_set_wall(robot_row, robot_col,
                  _abs_dir_front(robot_heading), tof_wall_front());
    maze_set_wall(robot_row, robot_col,
                  _abs_dir_right(robot_heading), tof_wall_right());

    // The direction we came from is open (we just traversed it)
    // — set on first move only; skip at start cell
    // (handled implicitly: if we came from south, that passage
    //  was already marked open when we left the previous cell)
}

// Execute a turn + forward toward `target_dir`, update position
static void _navigate_to_direction(uint8_t target_dir) {
    // Relative turn = (target - heading + 4) % 4
    uint8_t rel = (target_dir - (uint8_t)robot_heading + 4) & 0x03;

    switch (rel) {
        case 0: /* already facing */                           break;
        case 1: motion_turn_right();                           break;
        case 2: motion_turn_around();                          break;
        case 3: motion_turn_left();                            break;
    }

    // Update heading
    robot_heading = (Heading)target_dir;

    // Move forward one cell
    motion_forward(1);

    // Update position
    static const int8_t dRow[4] = { -1,  0,  1,  0 };
    static const int8_t dCol[4] = {  0,  1,  0, -1 };
    robot_row += dRow[target_dir];
    robot_col += dCol[target_dir];
}

// ══════════════════════════════════════════════════════════
//  Button reader — detects short-press, double-press, long-press
// ══════════════════════════════════════════════════════════
//
//  Returns:
//    0 = nothing
//    1 = short single press
//    2 = double press
//    3 = long press (held > BTN_LONG_PRESS_MS)
//
static uint8_t _read_button() {
    static bool     was_pressed   = false;
    static uint32_t press_start   = 0;
    static uint32_t last_release  = 0;
    static uint8_t  pending_click = 0;

    bool pressed = (digitalRead(PIN_BTN_START) == LOW);
    uint32_t now = millis();

    // ── Falling edge (button just pressed) ───────────────
    if (pressed && !was_pressed) {
        press_start = now;
        was_pressed = true;
        return 0;
    }

    // ── Held down — detect long-press ────────────────────
    if (pressed && was_pressed) {
        if ((now - press_start) >= BTN_LONG_PRESS_MS) {
            was_pressed = false;            // consume
            pending_click = 0;
            // Wait for release before returning
            while (digitalRead(PIN_BTN_START) == LOW) delay(10);
            return 3;  // LONG PRESS
        }
        return 0;
    }

    // ── Rising edge (button just released) ───────────────
    if (!pressed && was_pressed) {
        was_pressed = false;
        uint32_t held = now - press_start;
        if (held < BTN_DEBOUNCE_MS) return 0;      // bounce

        if (held < BTN_LONG_PRESS_MS) {
            // Short press — could be first of a double-press
            if (pending_click == 1 && (now - last_release) < BTN_DOUBLE_GAP_MS) {
                pending_click = 0;
                return 2;  // DOUBLE PRESS
            }
            pending_click = 1;
            last_release = now;
        }
        return 0;
    }

    // ── Idle — check if pending single-click has timed out ─
    if (pending_click == 1 && (now - last_release) >= BTN_DOUBLE_GAP_MS) {
        pending_click = 0;
        return 1;  // SINGLE SHORT PRESS
    }

    return 0;
}

// ══════════════════════════════════════════════════════════
//  CALIBRATE sub-state (button-driven)
// ══════════════════════════════════════════════════════════
//
//  Controls (single BOOT button):
//    Short press  → cycle to next test
//    Double press → execute current test
//    Long press   → exit calibration → IDLE
//
//  Test    LED colour    Action
//  ──────────────────────────────────────────────
//  0       Blue          Forward 1 cell (180 mm)
//  1       Yellow        360° rotation (4 × right)
//  2       Green         ToF wall check (LED flash)
//  3       Red           Square-up test
//
//  Serial 'C'/'c' from IDLE still enters calibration
//  (for when you DO have a laptop connected).

static uint8_t cal_idx = 0;   // currently selected test (0–3)

static void _calibrate_enter() {
    cal_idx = 0;
    led_set_cal_test(cal_idx);
    Serial.println(F("\n[CAL] ── Button Calibration Mode ──"));
    Serial.println(F("[CAL] Short press  = next test"));
    Serial.println(F("[CAL] Double press = run test"));
    Serial.println(F("[CAL] Long press   = exit"));
    Serial.println(F("[CAL] Test 0: Forward 1 cell (BLUE)"));
}

static void _calibrate_run_test() {
    // Brief white flash to confirm execution
    led_set(255, 255, 255);
    delay(150);

    switch (cal_idx) {
        case 0: {
            Serial.println(F("[CAL] ▶ Forward 1 cell..."));
            motion_forward(1);
            Serial.print(F("[CAL]   Encoder L="));
            Serial.print(encoder_get_left());
            Serial.print(F("  R="));
            Serial.println(encoder_get_right());
            break;
        }
        case 1: {
            Serial.println(F("[CAL] ▶ 360° rotation..."));
            motion_turn_right();
            motion_turn_right();
            motion_turn_right();
            motion_turn_right();
            Serial.println(F("[CAL]   360° complete. Check heading."));
            break;
        }
        case 2: {
            Serial.println(F("[CAL] ▶ ToF wall check..."));
            tof_update();
            bool wl = tof_wall_left();
            bool wf = tof_wall_front();
            bool wr = tof_wall_right();

            // Flash LED: bright = wall present, dim = absent
            //   Blue  = Left
            //   Green = Front
            //   Red   = Right
            led_flash_walls(wl, wf, wr);
            // Repeat so user can see clearly
            delay(300);
            led_flash_walls(wl, wf, wr);

            Serial.print(F("[CAL]   L="));
            Serial.print(tof_get_left());
            Serial.print(F("mm("));   Serial.print(wl ? "WALL" : "open");
            Serial.print(F(")  F=")); Serial.print(tof_get_front());
            Serial.print(F("mm("));   Serial.print(wf ? "WALL" : "open");
            Serial.print(F(")  R=")); Serial.print(tof_get_right());
            Serial.print(F("mm("));   Serial.print(wr ? "WALL" : "open");
            Serial.println(F(")"));
            break;
        }
        case 3: {
            Serial.println(F("[CAL] ▶ Square-up test..."));
            motion_square_up();
            break;
        }
        case 4: {
            Serial.println(F("[CAL] ▶ Motor self-test..."));
            Serial.println(F("[CAL]   Phase 1: LEFT motor only (1s)..."));
            led_set(0, 0, 200);   // blue = left
            encoder_reset();
            motor_set(150, 0);    // left only
            delay(1000);
            motor_brake();
            int64_t el = encoder_get_left();
            int64_t er = encoder_get_right();
            Serial.print(F("[CAL]   L_enc=")); Serial.print((long)el);
            Serial.print(F("  R_enc=")); Serial.println((long)er);
            if (abs(el) < 5) Serial.println(F("[CAL]   ⚠ LEFT MOTOR/ENCODER NOT WORKING!"));
            delay(500);

            Serial.println(F("[CAL]   Phase 2: RIGHT motor only (1s)..."));
            led_set(200, 0, 0);   // red = right
            encoder_reset();
            motor_set(0, 150);    // right only
            delay(1000);
            motor_brake();
            el = encoder_get_left();
            er = encoder_get_right();
            Serial.print(F("[CAL]   L_enc=")); Serial.print((long)el);
            Serial.print(F("  R_enc=")); Serial.println((long)er);
            if (abs(er) < 5) Serial.println(F("[CAL]   ⚠ RIGHT MOTOR/ENCODER NOT WORKING!"));
            delay(500);

            Serial.println(F("[CAL]   Phase 3: BOTH motors (1s)..."));
            led_set(0, 200, 0);   // green = both
            encoder_reset();
            motor_set(150, 150);  // both forward
            delay(1000);
            motor_brake();
            el = encoder_get_left();
            er = encoder_get_right();
            Serial.print(F("[CAL]   L_enc=")); Serial.print((long)el);
            Serial.print(F("  R_enc=")); Serial.println((long)er);
            Serial.println(F("[CAL]   Motor self-test complete."));
            break;
        }
    }

    // Restore test-colour pulsing
    led_set_cal_test(cal_idx);
}

static const char* _cal_test_name(uint8_t idx) {
    switch (idx) {
        case 0: return "Forward 1 cell (BLUE)";
        case 1: return "360 rotation (YELLOW)";
        case 2: return "ToF wall check (GREEN)";
        case 3: return "Square-up (RED)";
        case 4: return "Motor self-test (CYAN)";
        default: return "???";
    }
}

// ══════════════════════════════════════════════════════════
//  SOLVER INIT
// ══════════════════════════════════════════════════════════
void solver_init() {
    robot_row     = 0;
    robot_col     = 0;
    robot_heading = HEADING_N;
    state         = STATE_IDLE;

    maze_init();
    motion_init();

    Serial.println(F("[SOL] Solver initialised."));
    Serial.println(F("[SOL]   Short press  = START EXPLORE"));
    Serial.println(F("[SOL]   Long press   = CALIBRATION MODE"));
    Serial.println(F("[SOL]   Serial 'C'   = CALIBRATION MODE"));
}

// ══════════════════════════════════════════════════════════
//  SOLVER STEP — called every loop() tick
// ══════════════════════════════════════════════════════════
void solver_step() {
    switch (state) {

    // ── IDLE ─────────────────────────────────────────────
    case STATE_IDLE: {
        // Check for serial 'C' → calibration (backward compat)
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'C' || c == 'c') {
                state = STATE_CALIBRATE;
                _calibrate_enter();
                return;
            }
        }

        // Button: short = explore, long = calibrate
        uint8_t btn = _read_button();
        if (btn == 1 || btn == 2) {
            // Any short/double press → start exploring
            Serial.println(F("[SOL] Button pressed — starting EXPLORE"));
            state = STATE_EXPLORE;

            tof_update();
            _read_and_set_walls();
        } else if (btn == 3) {
            // Long press → calibration
            state = STATE_CALIBRATE;
            _calibrate_enter();
        }
        break;
    }

    // ── CALIBRATE ────────────────────────────────────────
    case STATE_CALIBRATE: {
        // Also allow serial commands as fallback
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'Q' || c == 'q') {
                Serial.println(F("[CAL] Exit → IDLE"));
                state = STATE_IDLE;
                return;
            }
        }

        uint8_t btn = _read_button();
        if (btn == 1) {
            // Short press → next test
            cal_idx = (cal_idx + 1) % CAL_NUM_TESTS;
            led_set_cal_test(cal_idx);
            Serial.print(F("[CAL] Test "));
            Serial.print(cal_idx);
            Serial.print(F(": "));
            Serial.println(_cal_test_name(cal_idx));
        } else if (btn == 2) {
            // Double press → run test
            _calibrate_run_test();
        } else if (btn == 3) {
            // Long press → exit
            Serial.println(F("[CAL] Exit → IDLE"));
            state = STATE_IDLE;
        }
        break;
    }

    // ── EXPLORE ──────────────────────────────────────────
    case STATE_EXPLORE: {
        // 1. Read walls at current cell
        _read_and_set_walls();

        // 2. Square up if front wall present (heading drift mitigation)
        if (tof_wall_front()) {
            motion_square_up();
        }

        // 3. Flood fill toward goal
        maze_flood_fill(true);

        // 4. Debug output
        Serial.print(F("[SOL] Pos=("));
        Serial.print(robot_row); Serial.print(',');
        Serial.print(robot_col); Serial.print(F(") H="));
        Serial.print((int)robot_heading);
        Serial.print(F(" Flood="));
        Serial.println(maze_get_flood(robot_row, robot_col));

        // 5. Check if we reached the goal
        if (_in_goal()) {
            Serial.println(F("[SOL] ★ GOAL REACHED — switching to RETURN"));
            led_set(0, 255, 0);
            delay(1000);
            state = STATE_RETURN;
            return;
        }

        // 6. Choose best direction and go
        uint8_t best = maze_best_direction(robot_row, robot_col);
        _navigate_to_direction(best);

        break;
    }

    // ── RETURN ───────────────────────────────────────────
    case STATE_RETURN: {
        // Read walls while returning (improve map)
        _read_and_set_walls();

        if (tof_wall_front()) {
            motion_square_up();
        }

        // Flood fill from start
        maze_flood_fill(false);

        Serial.print(F("[SOL] RETURN Pos=("));
        Serial.print(robot_row); Serial.print(',');
        Serial.print(robot_col); Serial.print(F(") Flood="));
        Serial.println(maze_get_flood(robot_row, robot_col));

        if (_at_start()) {
            Serial.println(F("[SOL] ★ BACK AT START — preparing SPEEDRUN"));
            motor_brake();

            // Flash LED, wait
            for (int i = 0; i < 6; i++) {
                led_set(255, 255, 0);
                delay(200);
                led_set(0, 0, 0);
                delay(200);
            }
            delay(2000);

            // Reset heading for speed run (robot should be at (0,0) facing North)
            robot_heading = HEADING_N;

            state = STATE_SPEEDRUN;
            return;
        }

        uint8_t best = maze_best_direction(robot_row, robot_col);
        _navigate_to_direction(best);

        break;
    }

    // ── SPEEDRUN ─────────────────────────────────────────
    case STATE_SPEEDRUN: {
        _read_and_set_walls();

        if (tof_wall_front()) {
            motion_square_up();
        }

        maze_flood_fill(true);

        Serial.print(F("[SOL] SPEED Pos=("));
        Serial.print(robot_row); Serial.print(',');
        Serial.print(robot_col); Serial.print(F(") Flood="));
        Serial.println(maze_get_flood(robot_row, robot_col));

        if (_in_goal()) {
            Serial.println(F("[SOL] ★★ SPEEDRUN COMPLETE ★★"));
            motor_brake();
            state = STATE_DONE;
            return;
        }

        // Navigate at higher speed
        uint8_t best = maze_best_direction(robot_row, robot_col);

        // Compute relative turn
        uint8_t rel = (best - (uint8_t)robot_heading + 4) & 0x03;
        switch (rel) {
            case 0:                          break;
            case 1: motion_turn_right();     break;
            case 2: motion_turn_around();    break;
            case 3: motion_turn_left();      break;
        }
        robot_heading = (Heading)best;

        // Forward at speed-run speed
        motion_forward_speed(1, SPEEDRUN_SPEED_COUNTS);

        // Update position
        static const int8_t dR[4] = { -1,  0,  1,  0 };
        static const int8_t dC[4] = {  0,  1,  0, -1 };
        robot_row += dR[best];
        robot_col += dC[best];

        break;
    }

    // ── DONE ─────────────────────────────────────────────
    case STATE_DONE:
        // Nothing to do — LED animation handled by led_update()
        break;

    } // end switch
}

RobotState solver_get_state()   { return state; }
uint8_t    solver_get_row()     { return robot_row; }
uint8_t    solver_get_col()     { return robot_col; }
Heading    solver_get_heading() { return robot_heading; }
