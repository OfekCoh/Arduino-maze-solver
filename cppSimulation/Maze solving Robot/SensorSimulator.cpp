#include "SensorSimulator.h"


// check if  a point is inside the maze
bool  SensorSimulator::isValid(const std::vector<uint8_t>& point) {
	return 0 <= point[1] && point[1] < MAZE_HEIGHT && 0 <= point[0] && point[0] < MAZE_WIDTH;
}

//left sensor
bool SensorSimulator::left_sensor(const vector<uint8_t>& currentLocation, int currentDirection, const vector<vector<uint8_t>>& maze) {
	vector<uint8_t> point_to_check(2);
	switch (currentDirection) {
	case NORTH:
		point_to_check = currentLocation;
		point_to_check[0]--; // the cell left to the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;
	case SOUTH:
		point_to_check = currentLocation;
		point_to_check[0]++; // the cell left to the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;
	case EAST:
		point_to_check = currentLocation;
		point_to_check[1]++; // the cell left to the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;

	case WEST:
		point_to_check = currentLocation;
		point_to_check[1]--; // the cell left to the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;


	default:
		break;
	}
	return false; // no wall was found
}


//front sensor
bool SensorSimulator::front_sensor(const vector<uint8_t>& currentLocation, int currentDirection, const vector<vector<uint8_t>>& maze) {
	vector<uint8_t> point_to_check(2);
	switch (currentDirection) {
	case NORTH:
		point_to_check = currentLocation;
		point_to_check[1]++; // the cell in front of the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;
	case SOUTH:
		point_to_check = currentLocation;
		point_to_check[1]--; // the cell in front of the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;
	case EAST:
		point_to_check = currentLocation;
		point_to_check[0]++; // the cell in front of the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;

	case WEST:
		point_to_check = currentLocation;
		point_to_check[0]--; // the cell in front of the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;


	default:
		break;
	}
	return false; // no wall was found
}

//right sensor
bool SensorSimulator::right_sensor(const vector<uint8_t>& currentLocation, int currentDirection, const vector<vector<uint8_t>>& maze) {
	vector<uint8_t> point_to_check(2);
	switch (currentDirection) {
	case NORTH:
		point_to_check = currentLocation;
		point_to_check[0]++; // the cell right to the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;
	case SOUTH:
		point_to_check = currentLocation;
		point_to_check[0]--; // the cell right to the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;
	case EAST:
		point_to_check = currentLocation;
		point_to_check[1]--; // the cell right to the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;

	case WEST:
		point_to_check = currentLocation;
		point_to_check[1]++; // the cell right to the car in the maze 
		if (isValid(point_to_check)) {
			if (maze[point_to_check[1]][point_to_check[0]] == 1) {// the sensor discovered a wall
				return true;
			}
		}
		break;


	default:
		break;
	}
	return false; // no wall was found
}



