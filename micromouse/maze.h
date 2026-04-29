#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>

// ── Maze Wall Map & Flood-Fill API ───────────────────────
//
// 16×16 cell grid.  Each cell stores a 4-bit wall bitmask:
//   bit 0 (0x01) = North    bit 1 (0x02) = East
//   bit 2 (0x04) = South    bit 3 (0x08) = West
//
// Flood-fill stores a distance value per cell (BFS from goal).

// Direction constants (used by solver too)
#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

void    maze_init();

void    maze_set_wall(uint8_t row, uint8_t col, uint8_t dir, bool present);
bool    maze_has_wall(uint8_t row, uint8_t col, uint8_t dir);

void    maze_flood_fill(bool to_goal);
uint8_t maze_get_flood(uint8_t row, uint8_t col);
void    maze_set_flood(uint8_t row, uint8_t col, uint8_t value);

uint8_t maze_best_direction(uint8_t row, uint8_t col);

// Debug helper — prints flood map to Serial
void    maze_print_flood();
void    maze_print_walls();

#endif // MAZE_H
