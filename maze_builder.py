# build maze using dpf, the maze will start with everythings a wall, using dfs to randomly "carve" the maze 
import random
from constants import MAZE_HEIGHT, MAZE_WIDTH, DIRECTIONS


# check if a position is within maze bounds
def is_valid(x, y):
    return 0 <= x < MAZE_WIDTH and 0 <= y < MAZE_HEIGHT

# dfs maze generator
def generate_maze():
    
    maze = [[1] * MAZE_WIDTH for _ in range(MAZE_HEIGHT)] # initialize the maze filled with walls (1)
    visited = [[False] * MAZE_WIDTH for _ in range(MAZE_HEIGHT)] # create a visited matrix to track visited cells
    
    # choose a random starting cell, ensuring it's not on the boundary
    start_cell = (0,0)
    
    # stack for dfs
    stack = [(start_cell[0], start_cell[1])]
    visited[start_cell[1]][start_cell[0]] = True
    maze[start_cell[1]][start_cell[0]] = 0  # mark the start cell as open
    
    # dfs to carve the maze
    while stack:
        x, y = stack[-1]  # current cell is at the top of the stack
        neighbors = [] # resets neighbors
        
        # look for unvisited neighbors
        for dx, dy in DIRECTIONS:
            nx, ny = x + dx * 2, y + dy * 2  # move two cells at a time to avoid removing wrong walls
            if is_valid(nx, ny) and not visited[ny][nx]: # if the cell is ini the maze and is not visited, add to visited matrix
                neighbors.append((nx, ny))
        
        if neighbors:
            # choose random unvisited neighbor if there is any
            nx, ny = random.choice(neighbors)
            
            # check which neighbor is it (by direction) and remove the wall between the cell and the neighbor
            if nx == x + 2:  # right
                maze[y][x + 1] = 0
            elif nx == x - 2:  # left
                maze[y][x - 1] = 0
            elif ny == y + 2:  # down
                maze[y + 1][x] = 0
            elif ny == y - 2:  # up
                maze[y - 1][x] = 0
            
            # mark the chosen neighbor as visited and add it to the stack
            visited[ny][nx] = True
            maze[ny][nx] = 0  # open the cell in the maze (0)
            stack.append((nx, ny))  # push the chosen neighbor to the stack
        else:
            # if no unvisited neighbors, pop the stack
            stack.pop()
    
    # for the exit, well take the cell thats open and furthest away from the start
    max_sum = 0
    for y in range(MAZE_HEIGHT):  
        for x in range(MAZE_WIDTH):  
            if maze[y][x] == 0:  # only consider cells that are 0, because we used dfs there is aroute to those cells
                current_sum = x + y 
                
                # track the cell with the highest index sum (furthest from teh start)
                if current_sum > max_sum:
                    max_sum = current_sum
                    exit_cell = (x, y)  # Store the coordinates of the cell with the highest sum
    
    # Return the generated maze and the starting point
    return maze, start_cell, exit_cell

# maze, start, exit_point = generate_maze()
# print("Maze generated with DFS:")
# for row in maze:
#     print(" ".join(str(cell) for cell in row))

# print(f"Start point: {start}")
# print(f"Exit point: {exit_point}")
