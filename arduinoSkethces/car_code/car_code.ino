#include <AFMotor.h> // Include motor shield library
#include <ArduinoSTL.h> // to use stl functions
#include <vector> // for matrix
#include <queue> // queue for bfs
#include <cstdint> // for uint8_t

// for comfort
using std::queue;
using std::vector;

// Create motor objects
AF_DCMotor motor1(4); // Front-Left Motor
AF_DCMotor motor2(3); // Front-Right Motor
AF_DCMotor motor3(1); // Rear-Left Motor
AF_DCMotor motor4(2); // Rear-Right Motor

// Ultrasonic Sensor Pins
const int trigFront = A0;
const int echoFront = A1;

const int trigLeft = A4;
const int echoLeft = A5;

const int trigRight = A2;
const int echoRight = A3;

///////////////////////// constants
// distance from sensor to determine wall (in cm)
#define WALL_DISTANCE  5

// movement duration
#define ROTATION_DURATION 2000
#define FORWARD_DURATION 500

// maze dimantions
#define MAZE_HEIGHT 10
#define MAZE_WIDTH 10

// directions
#define NORTH 1
#define SOUTH 2
#define WEST 3
#define EAST 4

const int directions[4][2] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} }; // directions to check around the current cell
///////////////////////// end of constants


// arduino Setup
void setup() {
  // Initialize motors
  motor1.setSpeed(100); // Speed ranges from 0-255
  motor2.setSpeed(100);
  motor3.setSpeed(100);
  motor4.setSpeed(100);

  // Initialize ultrasonic sensor pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  Serial.begin(9600); // For debugging
}

///////////////////////////// arduino functions
// Function to measure distance from ultrasonic sensor
long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration * 0.034) / 2; // Convert to cm
  return distance;
}


// Function to stop all motors
void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// fuction to turn around
void moveBackward() {
  Serial.println("forward ");
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(FORWARD_DURATION);
  stopMotors();
}

// Function to move forward
void moveForward() {
  Serial.println("forward ");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(FORWARD_DURATION);
  stopMotors();
}


// Function to rotate left
void rotateLeft() {
  Serial.println("left ");
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  delay(ROTATION_DURATION);
  stopMotors();
}

// Function to rotate right
void rotateRight() {
  Serial.println("right ");
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  delay(ROTATION_DURATION);
  stopMotors();
}
 bool turn = true;
///////////////////////////// end of arduino functions

//////////////////////////// functions
// check if  a point is inside the maze
bool isValid(const std::vector<uint8_t>& point) {
    return 0 <= point[1] && point[1] < MAZE_HEIGHT && 0 <= point[0] && point[0] < MAZE_WIDTH;
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

// handle sensors reading and update robot maze if needed
bool find_new_walls(long frontDistance, long rightDistance, long leftDistance, vector<vector<uint8_t>>& robot_maze, const vector<uint8_t>&  currentLocation, int currentDirection) {

    vector<uint8_t> new_wall;// new wall potential location 
    bool wall_found = false; // check if new wall was found

    
    // check left cell for wall
    if (leftDistance < WALL_DISTANCE) {
        new_wall = left_wall_location(currentLocation, currentDirection);
        if (isValid(new_wall)) {
            if (robot_maze[new_wall[1]][new_wall[0]] == 0) {//check if the robot doesnt know the wall already
                robot_maze[new_wall[1]][new_wall[0]] = 1;
                wall_found = true;
            }
        }
    }
    // check right cell for wall
    if (rightDistance < WALL_DISTANCE) {
        new_wall = right_wall_location(currentLocation, currentDirection);
        if (isValid(new_wall)) {
            if (robot_maze[new_wall[1]][new_wall[0]] == 0) {//check if the robot doesnt know the wall already
                robot_maze[new_wall[1]][new_wall[0]] = 1;
                wall_found = true;
            }
        }
    }
    // check front cell for wall
    if (frontDistance < WALL_DISTANCE) {
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



// move robot according to cells direction and current robots direction
void facingNORTH(int currentDirection, const vector<uint8_t>& currentLocation, const vector<uint8_t>& nextLocation) {
    if (currentLocation[0] < nextLocation[0]) {
        rotateRight();
        std::cout << "right" << std::endl;
    }
    if (currentLocation[0] > nextLocation[0]) {
        rotateLeft();
        std::cout << "left" << std::endl;
    }
    if (currentLocation[1] < nextLocation[1]) {
        moveForward();
        std::cout << "forward" << std::endl;
    }
    if (currentLocation[1] > nextLocation[1]) {
        moveBackward();
        std::cout << "back" << std::endl;
    }
}

void facingSOUTH(int currentDirection, const vector<uint8_t>& currentLocation, const vector<uint8_t>& nextLocation) {
    if (currentLocation[0] < nextLocation[0]) {
        rotateLeft();
        std::cout << "left" << std::endl;
    }
    if (currentLocation[0] > nextLocation[0]) {
        rotateRight();
        std::cout << "right" << std::endl;
    }
    if (currentLocation[1] < nextLocation[1]) {
        moveForward();
        std::cout << "back" << std::endl;
    }
    if (currentLocation[1] > nextLocation[1]) {
        moveBackward();
        std::cout << "forward" << std::endl;
    }
}

void facingEAST(int currentDirection, const vector<uint8_t>& currentLocation, const vector<uint8_t>& nextLocation) {
    if (currentLocation[0] < nextLocation[0]) {
        moveForward();
        std::cout << "forward" << std::endl;
    }
    if (currentLocation[0] > nextLocation[0]) {
        moveBackward();
        std::cout << "back" << std::endl;
    }
    if (currentLocation[1] < nextLocation[1]) {
        rotateLeft();
        std::cout << "left" << std::endl;
    }
    if (currentLocation[1] > nextLocation[1]) {
        rotateRight();
        std::cout << "right" << std::endl;
    }
}

void facingWEST(int currentDirection, const vector<uint8_t>& currentLocation, const vector<uint8_t>& nextLocation) {
    if (currentLocation[0] < nextLocation[0]) {
        moveBackward();
        std::cout << "back" << std::endl;
    }
    if (currentLocation[0] > nextLocation[0]) {
        moveForward();
        std::cout << "forward" << std::endl;
    }
    if (currentLocation[1] < nextLocation[1]) {
        rotateRight();
        std::cout << "right" << std::endl;
    }
    if (currentLocation[1] > nextLocation[1]) {
        rotateLeft();
        std::cout << "left" << std::endl;
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
    if (currentLocation[1] > nextLocation[1]) {
        return SOUTH;
    }
}

/////////////////////////// end of functions







//////////////////////////// global variables

// start and finish location
vector<uint8_t> start = { 0,0 };
vector<uint8_t> target = { 8,8 };

vector<vector<uint8_t>> robot_maze(MAZE_HEIGHT, vector<uint8_t>(MAZE_WIDTH, 0));// initilize the robots maze to be without walls 

bool walls_found = false;// to check if new walls was found this iteration, to update distances
vector<uint8_t> current_position = start; // the robot starting position

vector<vector<uint8_t>> distances(MAZE_HEIGHT, vector<uint8_t>(MAZE_WIDTH, 0)); // create the distances matrix, and update it with the current maze according to teh robot
//calculate_distance(robot_maze, target, distances);

int current_direction = NORTH;// the robot starting facing north
int min_distance = -1; // to find the neighbor with the lowest distance to move to, -1 for not initilized

vector<uint8_t> neighbor(2);// initilize neighbors point vectors
vector<uint8_t> min_distance_neighbor(2);

/////////////////////////// end of global variables

// main loop
void loop() {
  // Measure distances
  long frontDistance = measureDistance(trigFront, echoFront);
  long leftDistance = measureDistance(trigLeft, echoLeft);
  long rightDistance = measureDistance(trigRight, echoRight);

  // Debugging distances
  Serial.print("Front: ");  Serial.print(frontDistance); Serial.print(" cm, ");
  Serial.print("Left: ");   Serial.print(leftDistance); Serial.print(" cm, ");
  Serial.print("Right: ");  Serial.println(rightDistance);
  
  if (current_position == target) {// finish if the robot reached target
    return;
  }

  // check for walls using the sensors and update maze 
  walls_found = find_new_walls(frontDistance, rightDistance, leftDistance, robot_maze, current_position, current_direction);

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
      // move robot according to its current duration
      switch (current_direction){
      case NORTH:
          facingNORTH(current_direction, current_position, min_distance_neighbor);
          break;

      case SOUTH:
          facingSOUTH(current_direction, current_position, min_distance_neighbor);
          break;

      case WEST:
          facingWEST(current_direction, current_position, min_distance_neighbor);
          break;
      
      case EAST:
          facingEAST(current_direction, current_position, min_distance_neighbor);
          break;
      default:
          break;
      }

      ////////////////////////////////////// for debug
      // send the robot maze using serial bluetooth the maze with 3 as the current robot location
      for (int i = MAZE_HEIGHT - 1; i >= 0; i--)
      {
        for (int j = 0; j < MAZE_WIDTH; j++) {
            if (current_position[0] == j && current_position[1] == i) {
                Serial.print("3 ");
            }
            else {
                if (robot_maze[i][j] == 1) {
                    Serial.print("1 ");
                }
                else {
                    Serial.print("0 ");
                }
            }
        }
        Serial.println();
      }
      Serial.println();
      Serial.println();
      Serial.println();
      ///////////////////////////////////// end of debug
      
      current_direction = new_direction(current_position, min_distance_neighbor); // update direction
      current_position = min_distance_neighbor;// move robot to the neighbor thats closest to the target in the robots matrix
    }       
    
    min_distance = -1; // resets the minimum distance for next iteration
    delay(10); // Small delay for stability
 
}
