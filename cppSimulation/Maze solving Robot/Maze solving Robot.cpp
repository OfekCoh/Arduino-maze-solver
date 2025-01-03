
#include <iostream>
#include <vector> // for matrix
#include <queue> // queue for bfs
#include <cstdint> // for uint8_t
#include "Constants.h"
#include "SensorSimulator.h"
#include <cstdint>




// for comfort
using std::queue;
using std::vector;

//temp test
// if wall was found on the right
vector<uint8_t> right_wall_location(const vector<uint8_t>& currentLocation, int currentDirection);
// if wall was found on the left
vector<uint8_t> left_wall_location(const vector<uint8_t>& currentLocation, int currentDirection);
// if wall was found on the front
vector<uint8_t> front_wall_location(const vector<uint8_t>& currentLocation, int currentDirection);

// calculate distance from all cells to target using bfs (no wall between cells is like edge between vertices)
void calculate_distance(const vector<vector<uint8_t>>& maze, const vector<uint8_t>& target, vector<vector<uint8_t>>& distances);

// the robot movement algorithm, map the maze and move towards target using the cell with the minimum distance from the target each iteration
void flood_fill(const vector<vector<uint8_t>>& maze, vector<vector<uint8_t>>& robot_maze, const vector<uint8_t>& start, const vector<uint8_t>& target);

// handle sensors reading and update robot maze if needed
bool find_new_walls(vector<vector<uint8_t>>& robot_maze, const vector<uint8_t>& currentLocation, int currentDirection, const vector<vector<uint8_t>>& maze);

// return the new direction after movement
int new_direction(const vector<uint8_t>& currentLocation, const vector<uint8_t>& nextLocation);

// check if  a point is inside the maze
bool isValid(const std::vector<uint8_t>& point) {
    return 0 <= point[1] && point[1] < MAZE_HEIGHT && 0 <= point[0] && point[0] < MAZE_WIDTH;
}

int main()
{
    // define a 10x10 maze using vector initialization
    vector<std::vector<uint8_t>> maze = {
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 0, 0, 0, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 1, 1, 1, 1, 0, 1},
        {1, 0, 1, 0, 1, 0, 0, 0, 0, 1},
        {1, 0, 1, 1, 1, 1, 1, 1, 0, 1},
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 1, 1, 0, 0, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };

    

    vector<uint8_t> start = { 2,1 };
    vector<uint8_t> target = { 8,8 };
    
    // initilize the robots maze to be without walls 
    vector<vector<uint8_t>> robot_maze(MAZE_HEIGHT, vector<uint8_t>(MAZE_WIDTH, 0));
    //find_new_walls(robot_maze, start, EAST, maze);
    flood_fill(maze, robot_maze, start, target);
    
    
    //// print the maze
    for (const auto& row : maze) {
        for (uint8_t cell : row) {
            std::cout << (cell == 1 ? "1" : "0") << " "; // Pruint8_t '#' for walls and ' ' for paths
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;

    
    // print the maze
    for (const auto& row : robot_maze) {
        for (uint8_t cell : row) {
            std::cout << (cell == 1 ? "1" : "0") << " "; // Pruint8_t '#' for walls and ' ' for paths
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;

    
    return 0;
}




// calculate distance from all cells to target using bfs (no wall between cells is like edge between vertices)
void calculate_distance(const vector<vector<uint8_t>>& maze, const vector<uint8_t>& target, vector<vector<uint8_t>>& distances) {

    uint8_t current_distance; //used to keep current cells distance to calculate next cells
    vector<uint8_t> current_point(2);//the points well be checking 
    vector<uint8_t> neighbor(2);

    // reset distances matrix, 5 mark unvisited cells
    for (auto& row : distances) {
        std::fill(row.begin(), row.end(), 5);
    }
    queue<vector<uint8_t>> bfs_queue; // a queue of cell coardinets (2 size vector), to manage searched cells

    distances[target[1]][target[0]] = 0; // the distance from the target to itself
    bfs_queue.push(target);


    //bfs loop
    while (!bfs_queue.empty()) {

        current_point = bfs_queue.front(); // retrive the current point to calculate
        bfs_queue.pop();
        current_distance = distances[current_point[1]][current_point[0]];// current point distance from target used to calculate the distance form its neighbors

        // check every cell's neighbors
        for (auto direction : directions) {

            neighbor[0] = current_point[0] + direction[0];
            neighbor[1] = current_point[1] + direction[1];
            if (isValid(neighbor)) { // if the neighbor is inside the maze

                // if the neighbor isnt a wall and unvisited
                if (maze[neighbor[1]][neighbor[0]] == 0 && distances[neighbor[1]][neighbor[0]] == 5) {
                    distances[neighbor[1]][neighbor[0]] = current_distance + 1;
                    bfs_queue.push(neighbor);
                }

            }
        }
    }
}
// the robot movement algorithm, map the maze and move towards target using the cell with the minimum distance from the target each iteration
void flood_fill(const vector<vector<uint8_t>>& maze, vector<vector<uint8_t>>& robot_maze, const vector<uint8_t>& start, const vector<uint8_t>& target){

    bool walls_found = false;// to check if new walls was found this iteration, to update distances
    vector<uint8_t> current_position = start; // the robot starting position
    
    vector<vector<uint8_t>> distances(MAZE_HEIGHT, vector<uint8_t>(MAZE_WIDTH, 0)); // create the distances matrix, and update it with the current maze according to teh robot
    calculate_distance(robot_maze, target, distances);

    int current_direction = NORTH;// the robot starting facing north
    int min_distance = -1; // to find the neighbor with the lowest distance to move to, -1 for not initilized
    
    vector<uint8_t> neighbor(2);// initilize neighbors point vectors
    vector<uint8_t> min_distance_neighbor(2);

    //robot movement loop, untill it reaches target
    while (true) {

        if (current_position == target) {// finish if the robot reached target
            return;
        }

        // check for walls using the sensors and update maze 
        walls_found = find_new_walls(robot_maze, current_position, current_direction, maze);
        
        // check cell's neighbors to decide where to move and find walls
        for (auto direction : directions) {
            neighbor[0] = current_position[0] + direction[0];
            neighbor[1] = current_position[1] + direction[1];
            if (isValid(neighbor)) { // if the neighbor is inside the maze

                // if the cell is a wall move to check othe cells
                if (robot_maze[neighbor[1]][neighbor[0]] != 1) {
                    // update the current position to be the free neighbor closest to the target
                    if (distances[neighbor[1]][neighbor[0]] < min_distance || min_distance == -1) {
                        min_distance = distances[neighbor[1]][neighbor[0]];
                        min_distance_neighbor = neighbor;
                    }
                }
            }
        }
        // update the distances matrix if new walls were found
        if (walls_found) {
            calculate_distance(robot_maze, target, distances);
            walls_found = false;
        }
        else {
            current_direction = new_direction(current_position, min_distance_neighbor); // update direction
            current_position = min_distance_neighbor;// move robot to the neighbor thats closest to the target
        }

        ///////////////////////////////////////////////////////////////////////////////// for testing
        // pruint8_t the maze with 3 as the current robot location
        for (uint8_t i = 0; i < MAZE_HEIGHT; i++)
        {
            for (uint8_t j = 0; j < MAZE_WIDTH; j++) {
                if (current_position[0] == j && current_position[1] == i) {
                    std::cout << "3 ";
                }
                else {
                    if (maze[i][j] == 1) {
                        std::cout << "1 ";
                    }
                    else {
                        std::cout << "0 ";
                    }
                }
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << (int)current_position[0] << "," << (int)current_position[1] << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
       
        ///////////////////////////////////////////////////////////////////////////////

        min_distance = -1; // resets the minimum distance for next iteration
    }
}

// return the new direction after movement
int new_direction(const vector<uint8_t>& currentLocation, const vector<uint8_t>& nextLocation) {
    if (currentLocation[0] < nextLocation[0]) {
        return EAST;
    }
    if (currentLocation[0] > nextLocation[0]) {
        return WEST;
    }
    if (currentLocation[1] < nextLocation[1]) {
        return NORTH;
    }
    if (currentLocation[1] < nextLocation[1]) {
        return SOUTH;
    }
}


// handle sensors reading and update robot maze if needed
bool find_new_walls(vector<vector<uint8_t>>& robot_maze, const vector<uint8_t>&  currentLocation, int currentDirection,const vector<vector<uint8_t>>& maze) {

    vector<uint8_t> new_wall;// new wall potential location 
    bool wall_found = false; // check if new wall was found

    
    // check left cell for wall
    if (SensorSimulator::left_sensor(currentLocation, currentDirection, maze)) {
        new_wall = left_wall_location(currentLocation, currentDirection);
        if (isValid(new_wall)) {
            if (robot_maze[new_wall[1]][new_wall[0]] == 0) {//check if the robot doesnt know the wall already
                robot_maze[new_wall[1]][new_wall[0]] = 1;
                wall_found = true;
            }
        }
    }
    // check right cell for wall
    if (SensorSimulator::right_sensor(currentLocation, currentDirection, maze)) {
        new_wall = right_wall_location(currentLocation, currentDirection);
        if (isValid(new_wall)) {
            if (robot_maze[new_wall[1]][new_wall[0]] == 0) {//check if the robot doesnt know the wall already
                robot_maze[new_wall[1]][new_wall[0]] = 1;
                wall_found = true;
            }
        }
    }
    // check front cell for wall
    if (SensorSimulator::front_sensor(currentLocation, currentDirection, maze)) {
        new_wall = front_wall_location(currentLocation, currentDirection);
        if (isValid(new_wall)) {
            if (robot_maze[new_wall[1]][new_wall[0]] == 0) {//check if the robot doesnt know the wall already
                robot_maze[new_wall[1]][new_wall[0]] = 1;
                wall_found = true;
            }
        }
    }
    return wall_found;
}




// return new wall location according to the sensors reading
// if wall was found on the left
vector<uint8_t> left_wall_location(const vector<uint8_t>& currentLocation, int currentDirection) {
    
    vector<uint8_t> new_wall(currentLocation);// the location of the new walls
    switch (currentDirection)
    {
    case NORTH:
        if (currentLocation[0] - 1 >= 0) {// uint cant be negative
            new_wall[0] -= 1;
        }
        return new_wall;
        break;
    
    case SOUTH:
        new_wall[0] += 1;
        return new_wall;
        break;
    
    case WEST:
        if (currentLocation[1] - 1 >= 0) {// uint cant be negative
            new_wall[1] -= 1;
        }
        return new_wall;
        break;
    case EAST:
        new_wall[1] += 1;
        return new_wall;
        break;    
    default:
        break;
    }
}

// if wall was found on the right
vector<uint8_t> right_wall_location(const vector<uint8_t>& currentLocation, int currentDirection) {

    vector<uint8_t> new_wall(currentLocation);// the location of the new walls
    switch (currentDirection)
    {
    case SOUTH:
        if (currentLocation[0] - 1 >= 0) {// uint cant be negative
            new_wall[0] -= 1;
        }
        return new_wall;
        break;

    case NORTH:
        new_wall[0] += 1;
        return new_wall;
        break;

    case EAST:
        if (currentLocation[1] - 1 >= 0) {// uint cant be negative
            new_wall[1] -= 1;
        }
        return new_wall;
        break;
    case WEST:
        new_wall[1] += 1;
        return new_wall;
        break;
    default:
        break;
    }
}

// if wall was found in the front sensor
vector<uint8_t> front_wall_location(const vector<uint8_t>& currentLocation, int currentDirection) {

    vector<uint8_t> new_wall(currentLocation);// the location of the new walls
    switch (currentDirection)
    {
    case SOUTH:
        if (currentLocation[1] - 1 >= 0) {// uint cant be negative
            new_wall[1] -= 1;
        }
        return new_wall;
        break;

    case NORTH:
        new_wall[1] += 1;
        return new_wall;
        break;

    case WEST:
        if (currentLocation[0] - 1 >= 0) {// uint cant be negative
            new_wall[0] -= 1;
        }
        return new_wall;
        break;
    case EAST:
        new_wall[0] += 1;
        return new_wall;
        break;
    
    default:
        break;
    }
}









