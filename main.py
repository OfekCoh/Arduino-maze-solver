import pygame
import queue
from maze_builder import generate_maze
from flood_fill import flood_fill
from a_star import dynamic_a_star
from constants import MAZE_HEIGHT, MAZE_WIDTH
from time import sleep

# distinguish between algorithms for simulation
FLOOD_FILL = 'f'
A_STAR = 'a'

# constants for the screen
CELL_SIZE = 40  # size of each cell in the grid (pixels)
WINDOW_WIDTH = MAZE_WIDTH * CELL_SIZE  # width of the window
WINDOW_HEIGHT = MAZE_HEIGHT * CELL_SIZE  # height of the window

# colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (50, 50, 50)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# initialize pygame
pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Maze Visualization")
cloak = pygame.time.Clock()

# generate maze
maze, start , target = generate_maze()

# testing maze
# maze = [
#     [0, 1, 1, 1, 0, 0, 0, 0, 0, 1],
#     [0, 1, 1, 1, 0, 1, 0, 1, 0, 1],
#     [0, 1, 1, 1, 0, 1, 1, 1, 0, 1],
#     [0, 0, 0, 1, 1, 1, 0, 1, 0, 1],
#     [0, 0, 1, 0, 0, 0, 0, 1, 0, 1],
#     [0, 1, 1, 0, 1, 1, 0, 1, 0, 1],
#     [0, 0, 0, 0, 1, 1, 0, 1, 0, 1],
#     [0, 0, 1, 1, 1, 1, 0, 0, 0, 1],
#     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
#     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
# ]
# start = 0,0
# target = 4,2

# another testing maze
# maze = [
#     [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
#     [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1],
#     [1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1],
#     [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
#     [1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1],
#     [1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1],
#     [1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
#     [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
#     [1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
#     [1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1],
#     [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1],
#     [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1],
#     [1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
#     [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
#     [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1],
#     [1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
#     [1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
#     [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1],
#     [1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1],
#     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
# ]
# start = 0,0
# target = 10,10

# draw the maze
def draw_maze(distances = None):
    for y in range(MAZE_HEIGHT):
        for x in range(MAZE_WIDTH):
            rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            if (x,y) == target:
                pygame.draw.rect(screen, GREEN, rect)
            elif maze[y][x] == 1:
                pygame.draw.rect(screen, BLACK, rect)  # draw walls in black
            elif maze[y][x] == 2:
                pygame.draw.rect(screen, RED, rect)  # draw robot 
            else:
                pygame.draw.rect(screen, WHITE, rect)  # draw open paths in white
            
            # if the algorithm is flood fill, distances from target to the cells
            if distances != None:
                # Text parameters
                font = pygame.font.Font(None, 25)  # default font, size 50
                text = font.render(str(distances[y][x]), True, BLACK)  # render and color text
                text_rect = text.get_rect(center=rect.center)  # center the text on the rectangle
                screen.blit(text, text_rect)
            
            pygame.draw.rect(screen, GRAY, rect, 1)  # draw grid lines 


# simulate the robot solving the maze
def robot_movement_simulation(robot_locations_queue, distances_queue = None):

    pause = True # pause at key press
    
    # first step
    robot_location = robot_locations_queue.get() # moves the robot
    if distances_queue != None:# get the new distance matrix if flood fill    
        distance_matrix = distances_queue.get()
        
    while True:         
        
        # key pressed
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:  # check if a key was pressed
                if event.key == pygame.K_SPACE:  # pause and unpause if space was pressed
                    pause = not pause        
                if event.key == pygame.K_d:
                    pygame.quit()
        if pause == False:
            maze[robot_location[1]][robot_location[0]] = 0 # clean previous robot location  
            robot_location = robot_locations_queue.get() # moves the robot
            if distances_queue != None:# get the new distance matrix if flood fill    
                distance_matrix = distances_queue.get()
        
            
        # marks the robot location in this stage for printing
        if(robot_locations_queue.empty() == True): # the simulation has finished
             # reset maze
            maze[target[1]][target[0]] = 0
            maze[start[1]][start[0]] = 0 
            return
        
        maze[robot_location[1]][robot_location[0]] = 2 
        screen.fill((255, 255, 255))  # clear the screen with white
        
        
        # draw the maze, distance matrix will be null if a_star    
        if distances_queue == None:
            draw_maze()
        else:
            draw_maze(distance_matrix)
               
     
        
        
        pygame.display.flip()  # Update the display
        cloak.tick(10) # control fps 
    

# find shortest path using dynamic a*
def a_star_solve(robot_locations_queue, robot_maze):
    dynamic_a_star(maze, start, target, robot_maze,robot_locations_queue) # solve the maze using flood fill   
    robot_movement_simulation(robot_locations_queue)
    
    # iterate untill shortest path was found
    while True:   
        
        # solve the maze backwards to try and find better route
        dynamic_a_star(maze, target, start, robot_maze, robot_locations_queue) 
        route_length = robot_locations_queue.qsize() # the current length of the route 
        robot_movement_simulation(robot_locations_queue)
        
        # solve the maze again to try and find better route and finish at target
        dynamic_a_star(maze, start, target, robot_maze, robot_locations_queue) 
        
        # check if weve found the optimal route (the route doesnt improve after iterations)
        if route_length <= robot_locations_queue.qsize(): 
            robot_movement_simulation(robot_locations_queue)
            return
        robot_movement_simulation(robot_locations_queue)


# find shortest path using flood fill
def flood_fill_solve(robot_locations_queue, robot_maze):
    distances_queue = queue.Queue()
    flood_fill(maze, start, target, robot_maze, robot_locations_queue, distances_queue) # solve the maze using flood fill   
    robot_movement_simulation(robot_locations_queue, distances_queue)
    
    # iterate untill shortest path was found
    while True:   
        
        # solve the maze backwards to try and find better route
        flood_fill(maze, target, start, robot_maze, robot_locations_queue, distances_queue) 
        route_length = robot_locations_queue.qsize() # the current length of the route 
        robot_movement_simulation(robot_locations_queue, distances_queue)
        
        # solve the maze again to try and find better route and finish at target
        flood_fill(maze, start, target, robot_maze, robot_locations_queue, distances_queue) 
        
        # check if weve found the optimal route (the route doesnt improve after iterations)
        if route_length <= robot_locations_queue.qsize(): 
            robot_movement_simulation(robot_locations_queue, distances_queue)
            return
        robot_movement_simulation(robot_locations_queue, distances_queue)

# main loop to display the maze
def main():
    
    # choose which algorithm to use
    algorithm = input("choose an algorithm (a or f):")
    print("press space to pause/unpause")

    # solve the maze and keep the robot's route in a queue for visualization
    robot_locations_queue = queue.Queue()
    robot_maze = [[0] * MAZE_WIDTH for _ in range(MAZE_HEIGHT)]  # initilize robot maze matrix to 0 (unvisited cells) 
    
    # solve maze using chosen algorithm
    if algorithm == "f":
        flood_fill_solve(robot_locations_queue, robot_maze)
    else:
        a_star_solve(robot_locations_queue, robot_maze)

    pygame.quit() 

# Run the visualization
if __name__ == "__main__":
    main()
