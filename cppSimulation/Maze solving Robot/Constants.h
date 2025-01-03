#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <cstdint>
#include <vector>

using std::vector;

//constants
#define MAZE_HEIGHT 10
#define MAZE_WIDTH 10

#define NORTH 1
#define SOUTH 2
#define WEST 3
#define EAST 4

const int directions[4][2] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} }; // directions to check around the current cell



#endif
