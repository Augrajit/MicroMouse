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
//  CALIBRATE sub-state (serial-driven)
// ══════════════════════════════════════════════════════════

static void _calibrate_menu() {
    Serial.println(F("\n╔══════════════════════════════════╗"));
    Serial.println(F("║      CALIBRATION MODE            ║"));
    Serial.println(F("╠══════════════════════════════════╣"));
    Serial.println(F("║  1 — Forward 1 cell (180mm)      ║"));
    Serial.println(F("║  2 — 360° rotation (4×right)     ║"));
    Serial.println(F("║  3 — Print encoder/speed info    ║"));
    Serial.println(F("║  4 — Print ToF readings          ║"));
    Serial.println(F("║  5 — Test square-up              ║"));
    Serial.println(F("║  Q — Quit calibration → IDLE     ║"));
    Serial.println(F("╚══════════════════════════════════╝"));
}

static bool _calibrate_step() {
    if (!Serial.available()) return false;

    char c = Serial.read();

    switch (c) {
        case '1':
            Serial.println(F("[CAL] Forward 1 cell..."));
            motion_forward(1);
            Serial.print(F("[CAL] Encoder L="));
            Serial.print(encoder_get_left());
            Serial.print(F("  R="));
            Serial.println(encoder_get_right());
            break;

        case '2':
            Serial.println(F("[CAL] 360° rotation..."));
            motion_turn_right();
            motion_turn_right();
            motion_turn_right();
            motion_turn_right();
            Serial.println(F("[CAL] 360° complete. Check heading."));
            break;

        case '3':
            Serial.println(F("[CAL] ── Constants ──"));
            Serial.print(F("  GEAR_RATIO          = ")); Serial.println(GEAR_RATIO);
            Serial.print(F("  WHEEL_DIAMETER_MM   = ")); Serial.println(WHEEL_DIAMETER_MM);
            Serial.print(F("  WHEEL_BASE_MM       = ")); Serial.println(WHEEL_BASE_MM);
            Serial.print(F("  COUNTS_PER_WHEEL_REV= ")); Serial.println(COUNTS_PER_WHEEL_REV);
            Serial.print(F("  MM_PER_COUNT        = ")); Serial.println(MM_PER_COUNT, 5);
            Serial.print(F("  Counts/cell (180mm) = ")); Serial.println(CELL_SIZE_MM / MM_PER_COUNT, 1);
            Serial.print(F("  Counts/90° turn     = ")); Serial.println((PI * WHEEL_BASE_MM / 4.0f) / MM_PER_COUNT, 1);
            break;

        case '4':
            Serial.println(F("[CAL] ── ToF Readings ──"));
            tof_update();
            Serial.print(F("  Left  = ")); Serial.print(tof_get_left());  Serial.println(F(" mm"));
            Serial.print(F("  Front = ")); Serial.print(tof_get_front()); Serial.println(F(" mm"));
            Serial.print(F("  Right = ")); Serial.print(tof_get_right()); Serial.println(F(" mm"));
            Serial.print(F("  Wall L=")); Serial.print(tof_wall_left());
            Serial.print(F("  F=")); Serial.print(tof_wall_front());
            Serial.print(F("  R=")); Serial.println(tof_wall_right());
            break;

        case '5':
            Serial.println(F("[CAL] Square-up test..."));
            motion_square_up();
            break;

        case 'Q': case 'q':
            Serial.println(F("[CAL] Exiting calibration → IDLE"));
            state = STATE_IDLE;
            return true;

        default:
            _calibrate_menu();
            break;
    }
    return false;
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

    Serial.println(F("[SOL] Solver initialised. Waiting for button or 'C' for calibration."));
}

// ══════════════════════════════════════════════════════════
//  SOLVER STEP — called every loop() tick
// ══════════════════════════════════════════════════════════
void solver_step() {
    switch (state) {

    // ── IDLE ─────────────────────────────────────────────
    case STATE_IDLE: {
        // Check for serial 'C' → calibration
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'C' || c == 'c') {
                state = STATE_CALIBRATE;
                _calibrate_menu();
                return;
            }
        }

        // Wait for button press
        if (digitalRead(PIN_BTN_START) == LOW) {
            delay(50);   // debounce
            if (digitalRead(PIN_BTN_START) == LOW) {
                // Wait for release
                while (digitalRead(PIN_BTN_START) == LOW) delay(10);

                Serial.println(F("[SOL] Button pressed — starting EXPLORE"));
                state = STATE_EXPLORE;

                // Mark start cell: we know we came in from outside, so
                // the south wall of cell (0,0) is the outer boundary (already set).
                // Read initial walls at start cell.
                tof_update();
                _read_and_set_walls();
            }
        }
        break;
    }

    // ── CALIBRATE ────────────────────────────────────────
    case STATE_CALIBRATE:
        _calibrate_step();
        break;

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
