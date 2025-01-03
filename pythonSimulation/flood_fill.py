from collections import deque # for bfs
from constants import MAZE_HEIGHT, MAZE_WIDTH, DIRECTIONS


# calculate distance from all cells to target using bfs (no wall between cells is like edge between vertices)
def calculate_distances(maze, target):
   
    distances = [[-1] * MAZE_WIDTH for _ in range(MAZE_HEIGHT)]  # initialize distances matrix to -1 (unvisited cells)
    queue = deque([target]) # a double ended queue to iterate through the whole matrix
    distances[target[1]][target[0]] = 0  # distance at the target is 0

    while queue: # while there are cells to check around, check and update cells aroud queue[0]
        
        x, y = queue.popleft()
        current_distance = distances[y][x] # current cell distance to update nearby cells with

        for dx, dy in DIRECTIONS: 
            nx, ny = x + dx, y + dy # (nx, ny) is the cell next to current cell
            
            if is_valid(nx, ny):
                # only update if its an open cell (no wall) and it has not been updated yet
                if maze[ny][nx] == 0 and distances[ny][nx] == -1:
                    distances[ny][nx] = current_distance + 1
                    queue.append((nx, ny)) # add the cell to the queue to check the cells around it

    return distances # returns the distance matrix

# check if a cell
def is_valid(x, y):
    return 0 <= x < MAZE_WIDTH and 0 <= y < MAZE_HEIGHT

# the robot movement algorithm, explore the maze and move towards target
def flood_fill(maze, start, target, robot_maze, robot_locations_queue, distances_queue):
   
    current_position = start
    found_new_walls = False # if weve found new walls this iteration
    # calculate the distances from target in the current maze (the robot doesnt know where are the walls)
    distances = calculate_distances(robot_maze, target)
    
    min_distance = -1 # keep minimum neighbor distance to choose next step
    
    # insert first position to the robot route queue
    robot_locations_queue.put(current_position)
    distances_queue.put(distances)
    
    while True: # go until reaches target
        
        x, y = current_position
        
        # if the robot reaches the target finish algorithm 
        if current_position == target:
            return

        # Explore the neighbors and discover walls
        neighbors = []
        for dx, dy in DIRECTIONS:
            nx, ny = x + dx, y + dy          
            if is_valid(nx, ny):
                if maze[ny][nx] == 1:   # wall discovered
                    if robot_maze[ny][nx] == 0:# check if its new wall
                        robot_maze[ny][nx] = 1 # update robot maze
                        found_new_walls = True # weve found new walls so we need to update distance matrix
                    continue  # the cell is ureachable, dont check for movement
                neighbors.append((nx, ny))

        # move to the neighbor with the shortest distance to the target
        for neighbor in neighbors:
            if distances[neighbor[1]][neighbor[0]] <= min_distance or min_distance == -1:
                if neighbor == current_position and distances[neighbor[1]][neighbor[0]] == min_distance: # helps to prevent uneccecary backtracking
                    continue
                potential_current_position = neighbor
                min_distance = distances[neighbor[1]][neighbor[0]]
        
        current_position = potential_current_position # update current position
        min_distance = -1 # reset min_distance
        robot_locations_queue.put(current_position)
        distances_queue.put(distances)
        # if the robot discovered a wall during this iteration, recalculate distances for all cells
        if found_new_walls:
            distances = calculate_distances(robot_maze, target)
            found_new_walls = False
    

