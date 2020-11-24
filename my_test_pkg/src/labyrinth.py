#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import colors
# import os
import array
import map_saver 

class maze:
    def __init__(self, L):
        self.cmap1 = colors.ListedColormap(["white","black"])
        self.size = L

        # The final labyrinth will be stored in self.board
        self.board = np.ones([self.size,self.size])

        self.visited_cells = np.zeros([self.size,self.size])
        self.parent_x = np.zeros([self.size, self.size])
        self.parent_y = np.zeros([self.size, self.size])      
        self.i = 0
        
    def generate(self):
        """ Main-function for labyrinth-generation. """
        print(".:: Generating maze ::.")
        self.start_time = time.time()

        # Initiating current in [0,0] and setting its parent to itself. 
        self.current = np.array([0,0])
        self.parent_x[self.current[1], self.current[0]] = self.current[0]
        self.parent_y[self.current[1], self.current[0]] = self.current[1]
        
        self.board[self.current[0],self.current[1]] = 0
        self.log_visited()

        # Finding next movement, and performing the movement.
        self.move_to_next_cell(self.next_cell())
        self.log_visited()
        self.next_cell_var = self.next_cell()

        self.i = 0
        max_i = 300000
        
        while self.i < max_i:
            # Move randomly until there are no possible movements left. 
            while self.next_cell_var != -1 and self.i<max_i: 
                self.move_to_next_cell(self.next_cell_var)
                self.next_cell_var = self.next_cell()
                self.log_visited()
                
            # # When no more possible movements, go back one step
            self.go_back() 
            self.next_cell_var = self.next_cell() # check if there are any possible movements in this cell

            # If there still are no possible movements, keep going back until there are. 
            while self.next_cell_var == -1 and self.i > max_i: 
                self.go_back()
                self.next_cell_var = self.next_cell()
            
            # Check if current has returned to initial position, then the algorithm is done. 
            if int(self.current[0]) == 0 and int(self.current[1]) == 0:
                break
            
            self.i+=1 
            
            # Check if all cells has been visited. 
            if np.count_nonzero(self.visited_cells == 0) == 0:
                break

        print("- Maze created -")
        print("Time: "+str(round(time.time() - self.start_time,4))+" seconds")
        return self.board

    def log_visited(self):
        """ Keeping track of which cells are already path or wall """
        neighbours = [[-1,0],
                      [1,0],
                      [0,-1],
                      [0,1],
                       [1,1],
                       [1,-1],
                       [-1,1],
                       [-1,-1]]
        
        self.visited_cells[self.current[1], self.current[0]] = 1
        
        for n in neighbours:
            y = self.current[0] + n[0]
            x = self.current[1] + n[1]
            not_edge = x>=0 and x<self.size and y>=0 and y<self.size
            if not_edge:
                self.visited_cells[y, x] = 1
    
    def next_cell(self):
        """ Making a list of unexplored movements, then randomly selecting one from the list
            If no possible movements, return -1. This will initiate backtracing. 
        """

        # Since the walls occupy an entire block, each path must be atleast 3 blocks long. 
        directions = [[2,0],
                      [-2,0],
                      [0,2],
                      [0,-2]]
        
        possible_movements = []

        # Testing if direction is possible
        for d in directions:
            x = int(self.current[0] + d[0])
            y = int(self.current[1] + d[1])
            
            not_edge = x>=0 and x<self.size and y>=0 and y<self.size
            if not_edge:
                visited = not self.visited_cells[y, x]
                if visited:
                    possible_movements.append(d)
        
        if len(possible_movements) > 0:
            index = np.random.choice(range(len(possible_movements)))
            return possible_movements[index][:]

        else:
            return -1
                
    def go_back(self):
        """ Backtracing one step, by going to currents parent """
        pos = [int(self.current[0]), int(self.current[1])]
        self.current[0] = self.parent_x[pos[1], pos[0]]
        self.current[1] = self.parent_y[pos[1], pos[0]]
        
    def move_to_next_cell(self, next_cell):
        """ Moving current to the next cell, and filling in the maze in between """
        middle_x = int(self.current[0] + int(next_cell[0]/2))
        middle_y = int(self.current[1] + int(next_cell[1]/2))
 
        new_x = int(self.current[0] + next_cell[0])
        new_y = int(self.current[1] + next_cell[1])
        
        self.board[middle_y, middle_x] = 0
        self.board[new_y, new_x] = 0
        self.parent_x[new_y, new_x] = int(self.current[0])
        self.parent_y[new_y, new_x] = int(self.current[1])
        self.current = [new_x, new_y]
        
    def init_plot(self):
        """ Initiate fig1 and axs1 """
        self.fig1, self.axs1 = plt.subplots(1)

    def plot_maze(self):
        plt.cla()
        self.axs1.imshow(self.board, cmap = self.cmap1, interpolation = 'none')
        plt.show()
    
    def save_labyrinth(self):
        map_saver.save_map(self.board)

if __name__ == "__main__":
    L = 19
    maze_gen = maze(L)
    maze_gen.generate()
    maze_gen.init_plot()
    maze_gen.plot_maze()
    maze_gen.save_map()