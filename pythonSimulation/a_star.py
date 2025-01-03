import heapq
from constants import DIRECTIONS
from flood_fill import is_valid


# the heuristic function we used is manhattan distance (simple x-x + y-y)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# a* algoriathm to find the shortest path between current location and target (djikstra like)
def a_star_distance(maze, start, target):
    piority_queue = []  # priority queue (min-heap)
    came_from = {}  # for path reconstruction
    g_score = {start: 0}  # distance from the start to current node
    f_score = {start: heuristic(start, target)}  # distance from start to target through the node
    
    # push the start node to the open list
    heapq.heappush(piority_queue, (f_score[start], start))

    while piority_queue:
        # pop the node with the lowest f_score
        current_f, current = heapq.heappop(piority_queue)

        # if target is reached, reconstruct and return the path
        if current == target:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            # path.reverse()  # Reversing the path since it's built from target to start
            return path
        
        # explore neighbors
        for dx, dy in DIRECTIONS:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # check if neighbor is valid and open (0)
            if is_valid(neighbor[0], neighbor[1]) and maze[neighbor[1]][neighbor[0]] == 0:
                
                # potential g_score for this neighbor
                potential_g_score = g_score[current] + 1
                
                # if this is a shorter path to the neighbor update the route and the nodes g and f scores
                if neighbor not in g_score or potential_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = potential_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, target)
                    heapq.heappush(piority_queue, (f_score[neighbor], neighbor))

    return []  # if no path is found, return an empty list



# the robot movement algorithm, explore the maze and move towards target
def dynamic_a_star(maze, start, target, robot_maze, robot_locations_queue):
    
    found_new_walls = False # if weve found new walls this iteration
    route_to_target = a_star_distance(robot_maze, start, target) # find shortest route to target from current position

    # insert first position to the robot route queue
    robot_locations_queue.put(start)

    while True: # go until find exit
        
        current_position = route_to_target.pop()
        robot_locations_queue.put(current_position)      
        
        
        # if the robot reaches the target finish algorithm 
        if current_position == target:
            return

        # Explore the neighbors and discover walls
        for dx, dy in DIRECTIONS:
            nx, ny = current_position[0] + dx, current_position[1] + dy          
            if is_valid(nx, ny):
                if maze[ny][nx] == 1:   # wall discovered
                    if robot_maze[ny][nx] == 0:# check if its new wall
                        robot_maze[ny][nx] = 1 # update robot maze
                        found_new_walls = True # weve found new walls so we need to update distance matrix
                    continue  # the cell is ureachable, dont check for movement

       
        # if the robot discovered a wall during this iteration, calculate new route to target
        if found_new_walls:
            route_to_target = a_star_distance(robot_maze, current_position, target)
            found_new_walls = False
    




