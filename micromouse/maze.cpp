#include "maze.h"
#include "config.h"

// ── Internal data ────────────────────────────────────────
static uint8_t walls[MAZE_SIZE][MAZE_SIZE];  // wall bitmask per cell
static uint8_t flood[MAZE_SIZE][MAZE_SIZE];  // flood-fill distance

// Direction offsets: N, E, S, W → (dRow, dCol)
static const int8_t dRow[4] = { -1,  0,  1,  0 };
static const int8_t dCol[4] = {  0,  1,  0, -1 };

// Opposite direction: N↔S, E↔W
static inline uint8_t _opposite(uint8_t dir) {
    return (dir + 2) & 0x03;
}

// Wall bit for a given direction
static inline uint8_t _wall_bit(uint8_t dir) {
    return (1 << dir);
}

// ══════════════════════════════════════════════════════════
//  INIT
// ══════════════════════════════════════════════════════════
void maze_init() {
    // Clear all interior walls
    memset(walls, 0, sizeof(walls));
    memset(flood, 255, sizeof(flood));

    // Set outer boundary walls
    for (uint8_t i = 0; i < MAZE_SIZE; i++) {
        walls[0][i]             |= WALL_N;   // top row has north wall
        walls[MAZE_SIZE - 1][i] |= WALL_S;   // bottom row has south wall
        walls[i][0]             |= WALL_W;   // left column has west wall
        walls[i][MAZE_SIZE - 1] |= WALL_E;   // right column has east wall
    }

    Serial.println(F("[MAZE] Maze initialised (boundary walls set)"));
}

// ══════════════════════════════════════════════════════════
//  WALL ACCESS
// ══════════════════════════════════════════════════════════
void maze_set_wall(uint8_t row, uint8_t col, uint8_t dir, bool present) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE || dir > 3) return;

    if (present) {
        walls[row][col] |= _wall_bit(dir);
    } else {
        walls[row][col] &= ~_wall_bit(dir);
    }

    // Mirror on adjacent cell
    int8_t nr = row + dRow[dir];
    int8_t nc = col + dCol[dir];
    if (nr >= 0 && nr < MAZE_SIZE && nc >= 0 && nc < MAZE_SIZE) {
        uint8_t opp = _opposite(dir);
        if (present) {
            walls[nr][nc] |= _wall_bit(opp);
        } else {
            walls[nr][nc] &= ~_wall_bit(opp);
        }
    }
}

bool maze_has_wall(uint8_t row, uint8_t col, uint8_t dir) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE || dir > 3) return true;
    return (walls[row][col] & _wall_bit(dir)) != 0;
}

// ══════════════════════════════════════════════════════════
//  FLOOD FILL (BFS)
// ══════════════════════════════════════════════════════════
//
// Uses a simple circular queue in a static array (no dynamic
// allocation).  Queue size = 256 is enough for 16×16 = 256 cells.

#define QUEUE_SIZE 256

void maze_flood_fill(bool to_goal) {
    // Reset all distances to infinity
    memset(flood, 255, sizeof(flood));

    // Circular queue (stores packed row/col as single byte)
    static uint8_t q[QUEUE_SIZE];
    uint8_t head = 0, tail = 0;

    // Seed the queue with goal cells
    if (to_goal) {
        // Goal zone: rows GOAL_ROW_MIN..MAX, cols GOAL_COL_MIN..MAX
        for (uint8_t r = GOAL_ROW_MIN; r <= GOAL_ROW_MAX; r++) {
            for (uint8_t c = GOAL_COL_MIN; c <= GOAL_COL_MAX; c++) {
                flood[r][c] = 0;
                q[tail++] = (r << 4) | c;   // pack row:col into one byte
            }
        }
    } else {
        // Fill from start cell (0, 0)
        flood[0][0] = 0;
        q[tail++] = 0x00;
    }

    // BFS
    while (head != tail) {
        uint8_t packed = q[head++];   // auto-wraps at 256 for uint8_t
        uint8_t r = packed >> 4;
        uint8_t c = packed & 0x0F;
        uint8_t d = flood[r][c];

        for (uint8_t dir = 0; dir < 4; dir++) {
            if (maze_has_wall(r, c, dir)) continue;   // wall blocks passage

            int8_t nr = r + dRow[dir];
            int8_t nc = c + dCol[dir];
            if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;

            if (flood[nr][nc] > d + 1) {
                flood[nr][nc] = d + 1;
                q[tail++] = ((uint8_t)nr << 4) | (uint8_t)nc;
            }
        }
    }
}

uint8_t maze_get_flood(uint8_t row, uint8_t col) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return 255;
    return flood[row][col];
}

void maze_set_flood(uint8_t row, uint8_t col, uint8_t value) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return;
    flood[row][col] = value;
}

// ══════════════════════════════════════════════════════════
//  BEST DIRECTION — lowest-flood passable neighbor
// ══════════════════════════════════════════════════════════
uint8_t maze_best_direction(uint8_t row, uint8_t col) {
    uint8_t best_dir = DIR_N;
    uint8_t best_val = 255;

    for (uint8_t dir = 0; dir < 4; dir++) {
        if (maze_has_wall(row, col, dir)) continue;

        int8_t nr = row + dRow[dir];
        int8_t nc = col + dCol[dir];
        if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;

        if (flood[nr][nc] < best_val) {
            best_val = flood[nr][nc];
            best_dir = dir;
        }
    }

    return best_dir;
}

// ══════════════════════════════════════════════════════════
//  DEBUG
// ══════════════════════════════════════════════════════════
void maze_print_flood() {
    Serial.println(F("\n── Flood Map ──────────────────────"));
    for (uint8_t r = 0; r < MAZE_SIZE; r++) {
        for (uint8_t c = 0; c < MAZE_SIZE; c++) {
            if (flood[r][c] == 255)
                Serial.print(F(" -- "));
            else {
                if (flood[r][c] < 10) Serial.print(' ');
                Serial.print(flood[r][c]);
                Serial.print(' ');
            }
        }
        Serial.println();
    }
}

void maze_print_walls() {
    Serial.println(F("\n── Wall Map ───────────────────────"));
    // Print top border
    for (uint8_t c = 0; c < MAZE_SIZE; c++) Serial.print(F("+---"));
    Serial.println('+');

    for (uint8_t r = 0; r < MAZE_SIZE; r++) {
        // Row cells with east/west walls
        for (uint8_t c = 0; c < MAZE_SIZE; c++) {
            Serial.print(maze_has_wall(r, c, DIR_W) ? '|' : ' ');
            Serial.print(F("   "));
        }
        Serial.println(maze_has_wall(r, MAZE_SIZE - 1, DIR_E) ? '|' : ' ');

        // South walls
        for (uint8_t c = 0; c < MAZE_SIZE; c++) {
            Serial.print('+');
            Serial.print(maze_has_wall(r, c, DIR_S) ? F("---") : F("   "));
        }
        Serial.println('+');
    }
}
