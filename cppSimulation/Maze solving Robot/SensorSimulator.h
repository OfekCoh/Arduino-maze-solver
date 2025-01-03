// this file will simulate the sesors reading

#ifndef SENSORS_H
#define SENSORS_H

#include "Constants.h" // for constants  

//#include <cstdint>
using std::vector;

class SensorSimulator
{
public:
	//check each sensor
	bool static front_sensor(const vector<uint8_t>& currentLocation, int currentDirection, const vector<vector<uint8_t>>& maze);
	bool static right_sensor(const vector<uint8_t>& currentLocation, int currentDirection, const vector<vector<uint8_t>>& maze);
	bool static left_sensor(const vector<uint8_t>& currentLocation, int currentDirection, const vector<vector<uint8_t>>& maze);


private:
	// check if  a point is inside the maze
	bool static isValid(const std::vector<uint8_t>& point);
	
};

#endif
