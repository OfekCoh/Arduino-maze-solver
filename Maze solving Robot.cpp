
#include <iostream>
#include <vector> // for matrix
#include <deque> // double edge queue for bfs

// for comfort
using std::deque;
using std::vector;

//constants
#define MAZE_HEIGHT 10
#define MAZE_WIDTH 10

vector<vector<int>> directions = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} }; // directions to check around the current cell

// check if  a point is inside the maze
bool isValid(const vector<int>& point);

// calculate distance from all cells to target using bfs (no wall between cells is like edge between vertices)
void calculate_distance(const vector<vector<int>>& maze, const vector<int>& target, vector<vector<int>>& distances);

// the robot movement algorithm, map the maze and move towards target using the cell with the minimum distance from the target each iteration
void flood_fill(const vector<vector<int>>& maze, vector<vector<int>>& robot_maze, const vector<int>& start, const vector<int>& target);


int main()
{
    // define a 10x10 maze using vector initialization
    vector<std::vector<int>> maze = {
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 1, 1, 1, 1, 1, 0, 1},
        {1, 0, 1, 0, 0, 0, 0, 1, 0, 1},
        {1, 0, 1, 0, 1, 1, 1, 1, 0, 1},
        {1, 0, 1, 0, 1, 0, 0, 0, 0, 1},
        {1, 0, 1, 1, 1, 1, 1, 1, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };

    // print the maze
    for (const auto& row : maze) {
        for (int cell : row) {
            std::cout << (cell == 1 ? "1" : "0") << " "; // Print '#' for walls and ' ' for paths
        }
        std::cout << std::endl;
    }
    vector<int> start = { 0,0 };
    vector<int> target = { 8,8 };
    
    // initilize the robots maze to be without walls 
    vector<vector<int>> robot_maze(MAZE_HEIGHT, vector<int>(MAZE_WIDTH, 0)); 
    flood_fill(maze, robot_maze, start, target);
    return 0;
}




// calculate distance from all cells to target using bfs (no wall between cells is like edge between vertices)
void calculate_distance(const vector<vector<int>>& maze, const vector<int>& target, vector<vector<int>>& distances) {

    int current_distance; //used to keep current cells distance to calculate next cells
    vector<int> current_point(2);//the points well be checking 
    vector<int> neighbor(2);
    
    // reset distances matrix, -1 mark unvisited cells
    for (auto& row : distances) {      
        std::fill(row.begin(), row.end(), -1); 
    }
    deque<vector<int>> queue; // a queue of cell coardinets (2 size vector), to manage searched cells
    
    distances[target[1]][target[0]] = 0; // the distance from the target to itself
    queue.push_back(target);

    
    //bfs loop
    while (!queue.empty()){
        
        current_point = queue.back(); // retrive the current point to calculate
        queue.pop_back();
        current_distance = distances[current_point[1]][current_point[0]];// current point distance from target used to calculate the distance form its neighbors
        
        // check every cell's neighbors
        for (auto direction : directions) {
           
            neighbor[0] = current_point[0] + direction[0];
            neighbor[1] = current_point[1] + direction[1];
            if (isValid(neighbor)) { // if the neighbor is inside the maze
                
                // if the neighbor isnt a wall and unvisited
                if (maze[neighbor[1]][neighbor[0]] == 0 && distances[neighbor[1]][neighbor[0]] == -1) {
                    distances[neighbor[1]][neighbor[0]] = current_distance + 1;
                    queue.push_front(neighbor);
                }

            }
        }
    }


}

// the robot movement algorithm, map the maze and move towards target using the cell with the minimum distance from the target each iteration
void flood_fill(const vector<vector<int>>& maze, vector<vector<int>>& robot_maze, const vector<int>& start, const vector<int>& target) {
    
    bool walls_found = false;// to check if new walls was found this iteration, to update distances
    vector<int> current_position = start; // the robot starting position
    vector<vector<int>> distances(MAZE_HEIGHT, vector<int>(MAZE_WIDTH, 0)); // create the distances matrix, and update it with the current maze according to teh robot
    calculate_distance(robot_maze, target, distances);

    int min_distance = -1; // to find the neighbor with the lowest distance to move to, -1 for not initilized
    vector<int> neighbor(2);
    vector<int> min_distance_neighbor(2);

    //robot movement loop, untill it reaches target
    while (true) {
        
        if (current_position == target) {// finish if the robot reached target
            return;
        }
        
        // check cell's neighbors to decide where to move and find walls
        for (auto direction : directions) {
            neighbor[0] = current_position[0] + direction[0];
            neighbor[1] = current_position[1] + direction[1];
            if (isValid(neighbor)) { // if the neighbor is inside the maze

                // if a new wall was discovered update the robot's maze
                if (maze[neighbor[1]][neighbor[0]] == 1) { 
                    robot_maze[neighbor[1]][neighbor[0]] = 1;
                    walls_found = true;
                }
                else {
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
        
        current_position = min_distance_neighbor;// move robot to the neighbor thats closest to the target
        // for testing
        std::cout << current_position[0] << "," << current_position[1] << std::endl;
        min_distance = -1; // resets the minimum distance for next iteration
    }
}

// check if  a point is inside the maze
bool isValid(const vector<int>& point) {
    return 0 <= point[1] && point[1] < MAZE_HEIGHT && 0 <= point[0] && point[0] < MAZE_WIDTH;
}