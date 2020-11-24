#!/usr/bin/env python
import labyrinth
import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import colors
import copy

cmap1 = colors.ListedColormap(['white','black',(0.5,0.8,0.5),'orange','blue'])

class node:
    pass

class A_star:
    def __init__(self, input_map, liveplot = False):
        self.scale = 1
        self.liveplot = liveplot
        self.map_d = input_map
        self.final_path = []
        

        self.L = len(self.map_d)

        self.p_start_d = np.array([0,0])
        self.p_goal_d = np.array([self.L-1,self.L-1])
        self.start_time = 0
        
        # initialize open and closed list.
        self.open_nodes = []
        self.closed_nodes = []
        self.open_nodes2 = np.zeros([self.L, self.L])
        self.closed_nodes2 = np.zeros([self.L, self.L])

        self.f_cost2 = np.empty([self.L, self.L])
        self.f_cost2[:] = np.nan
        self.g_cost2 = np.ones([self.L, self.L])*self.L*self.L
        self.parents2 = np.zeros([self.L, self.L, 2])

        # set max iterations
        self.max_i = self.L*self.L
        self.path = np.zeros([self.max_i, 2])
        
        # Initiating plot
        self.fig1, self.axs1 = plt.subplots(1)
         
        self.q = 0
        self.closed_nodes_list = []
    
    def plot_map(self):
        """ plotting map """
        self.axs1.cla()
        self.axs1.set_xlim(-0.5, self.L-0.5)
        self.axs1.set_ylim(self.L-0.5, -0.5)
        self.axs1.imshow(self.map_d, 
                         vmin = 0, 
                         vmax = 4, 
                         cmap = cmap1, 
                         interpolation="nearest", 
                         label=["Blank Space", 
                                "Terrain", 
                                "Open", 
                                "Closed", 
                                "Optimal path"])
        self.axs1.plot(self.p_start_d[0], self.p_start_d[1],"go", label="Start")
        self.axs1.plot(self.p_goal_d[0], self.p_goal_d[1],"ro", label="Goal")
        self.axs1.legend()
        plt.pause(0.1)
        plt.show()
        
    
    def start_algorithm(self):
        """ Main algorithm function """
        self.start_time = time.time()
        start_node = node()
        start_node.xy = self.p_start_d
        start_node.g_cost = 0
        start_node.f_cost = 0
        start_node.parent = None

        i = 0
        self.parents2[int(self.p_start_d[1]),int(self.p_start_d[0])] = [None, None]
        self.open_nodes2[int(self.p_start_d[1]),int(self.p_start_d[0])] = 1
        self.g_cost2[int(self.p_start_d[1]),int(self.p_start_d[0])] = 0
        self.open_nodes.append(start_node)
        
        print(".:: Starting A* algorithm ::.") 
        
        while i < self.max_i:
            if i == 0:
                # Initiate current in starting-position
                current = [self.p_start_d[0], self.p_start_d[1]]     
            else:
                # Setting current to the neighbor with lowest f-cost
                current = np.unravel_index(np.nanargmin(self.f_cost2), self.f_cost2.shape)
                current = [current[1], current[0]]

            # Updating arrays keeping track of open nodes, closed nodes, f-cost and the map. 
            self.open_nodes2[current[1], current[0]] = 0
            self.closed_nodes2[current[1],current[0]] = 1
            self.f_cost2[current[1],current[0]] = np.nan
            self.map_d[current[1], current[0]] = 3

            # Check if goal has been reached
            if current[0] == self.p_goal_d[0] and current[1] == self.p_goal_d[1]:
                print("- Goal reached - ")
                self.total_time = time.time()-self.start_time
                print("Time: "+str(round(self.total_time, 3))+" seconds")
                self.find_final_path(current)
                break
            
            # finding neighbors of current
            neighbours = self.find_neighbours(current)

            # get g-cost of current. 
            current_g_cost = self.g_cost2[current[1], current[0]]
            
            # Iterating through neighbors. 
            for neighbour in neighbours:
                # Get g-cost of neighbor. 
                neighbour_g_cost = current_g_cost + self.distance_nodes(current, neighbour)
                
                # Check if neighbor is closed. 
                if self.closed_nodes2[neighbour[1], neighbour[0]]:
                    pass
                
                # If neighbor is not in the open-list, or if the new g-cost is lower than the old. 
                elif self.open_nodes2[neighbour[1], neighbour[0]] == 0 or neighbour_g_cost < self.g_cost2[neighbour[1], neighbour[0]]:
                    g_cost = neighbour_g_cost  
                    f_cost = self.get_f_cost(neighbour, g_cost)
                    self.map_d[neighbour[1], neighbour[0]] = 2
                    
                    # update f-cost, g-cost, and parent of neighbor...
                    # ...If neighbor is not opened yet
                    if self.open_nodes2[neighbour[1], neighbour[0]] == 0:
                        self.open_nodes2[neighbour[1], neighbour[0]] = 1
                        self.f_cost2[neighbour[1], neighbour[0]] = f_cost
                        self.g_cost2[neighbour[1], neighbour[0]] = g_cost
                        self.parents2[neighbour[1], neighbour[0]] = current
                        
                    # ...if new g-cost is lower than old
                    elif self.g_cost2[neighbour[1], neighbour[0]] > g_cost:
                        self.f_cost2[neighbour[1], neighbour[0]] = f_cost
                        self.g_cost2[neighbour[1], neighbour[0]] = g_cost
                        self.parents2[neighbour[1], neighbour[0]] = current
                   
            if self.liveplot:
                 self.plot_map()                                           
            
            if np.count_nonzero(self.open_nodes2 == 1) == 0:
                print("No solution")
                break
            i+=1
      
        
        if i == self.max_i:
            print("Maxed out i")
    

    def remove_current(self, current):
        """ closing the old current, by remocving it from open-list, and adding to closed-list """
        occurences = 0
        for n in self.open_nodes:
            if int(n.xy[0])==int(current.xy[0]) and int(n.xy[1]) == int(current.xy[1]):
                self.open_nodes.remove(n)
                occurences +=1
                self.closed_nodes2[int(current.xy[1]),int(current.xy[0])]=1
                
    
    def paint_open_green(self):
        """ Paint the open nodes green """
        for n in self.open_nodes:
            self.map_d[int(n.xy[1]), int(n.xy[0])] = 2
        
    
    def check_closed_nodes(self, n):
        """ Check if node n is closed """
        pos = n.xy

        if self.closed_nodes2[int(pos[1]), int(pos[0])]:
            return False

        return True       
    
    def generate_maze(self, size):
        """ generate a labyrinth """
        maze_obj = labyrinth.maze(size)
        self.map_d = maze_obj.generate()
        self.second_init()
    
    def find_final_path(self, last_node):
        """ Once goal has been reached --> backtrace through parents until starting position """
        pos = last_node
        self.final_path.insert(0, pos)
        parent = self.parents2[int(pos[1]), int(pos[0])]
        while not np.isnan(parent[0]):
            self.final_path.insert(0, parent)
            parent_pos = parent
            parent = self.parents2[int(parent_pos[1]),int(parent_pos[0])]
        
        for point in self.final_path:
            self.map_d[int(point[1]),int(point[0])]=4
            
    def print_node(self, n):
        """ Printing a node - for debugging purposes """
        print("position: ", n.xy)
        if n.parent == None:
            pass
        else: 
            print("parent: ", n.parent.xy)
        print("g-cost: ", n.g_cost)
        print("f-cost: ", n.f_cost)
        print("")
    
    def distance_nodes(self,a,b):
        """ Return distance between node a and b """
        distance = np.linalg.norm(np.array(a) - np.array(b))
        return distance
            
    def lowest_f_cost(self):
        """ returns the node with the lowest f-cost """
        lowest = float("Inf")
        output_node = node()
        output_index = -1
        
        for i in range(len(self.open_nodes)):           
            f_cost = self.open_nodes[i].f_cost
            if f_cost <= lowest:
                lowest = f_cost
                output_index = i
        
        return output_index
    
    def distance_to_goal(self, n):
        """ Calculates distance to goal from node """
        distance = np.linalg.norm(self.p_goal_d - n)
        return distance
    
    def get_f_cost(self, n, g_cost):
        """ Calculates f_cost of node """
        h_cost = self.distance_to_goal(n)
        f_cost = h_cost + g_cost
        return f_cost 
        
    def get_g_cost(self, n):
        """ Calculates g_cost """
        n_pos = n
        if np.isnan(self.parents2[int(n[1]),int(n[0])][0]):
            return 0
        else:
            p_pos = self.parents2[int(n[1]),int(n[0])]
             
            delta_x = n_pos[0] - p_pos[0]
            delta_y = n_pos[1] - p_pos[1] 
            distance = np.sqrt(delta_x**2 + delta_y**2)      
            
            g_cost = self.g_cost2[int(p_pos[1]), int(p_pos[0])] + distance
            return round(g_cost, 3)
        
    
    def find_neighbours(self, n):
        """ finds neighbours, and adds them to open(?) """
        pos = n
        neighbours = []
        coordinate_list = []

        # Set True to allow robot to move diagonally across cells. 
        diagonal_movements_allowed = False

        if diagonal_movements_allowed:
            coordinate_list = [[-1,0],
                            [-1,1],
                            [0,1],
                            [1,1],
                            [1,0],
                            [1,-1],
                            [0,-1],
                            [-1,-1]]
        else:
            coordinate_list = [[-1,0],
                            [0,1],
                            [1,0],
                            [0,-1]]
        
        # Iterating trough possible movements. 
        for n in coordinate_list:
            i = n[0]
            j = n[1]
            x = int(pos[0])
            y = int(pos[1])
    
            # Check if movement is within border of the map
            if x+i < self.L*self.scale and y+j < self.L*self.scale and x+i >=0 and y+j >=0:
                # Check if new position is free or terrain. 1 == terrain. 
                if self.map_d[y+j, x+i] != 1:
                    neighbours.append([x+i, y+j])
            
        return neighbours
                    
    def load_map(self, filepath):
        """ For importing a previously saved map """
        self.map_d = np.load(filepath)
        self.second_init()
    
    def return_direction_list(self):
        """ Converting the fial optimal path to a list of left/right instructions """
        self.directions = []

        first_vector = np.array([1,0])

        for i in range(0, len(self.final_path)-1):
            if i == 0:
                past_vector = first_vector
            else:
                past_vector = self.final_path[i] - self.final_path[i-1]

            next_vector = self.final_path[i+1] - self.final_path[i]

            # checking if vectors are not parallell, which means turning left/right.
            if past_vector.dot(next_vector)==0:

                # Calculating approximate angle between vectors. 
                # The only important thing is knowing the sign of the angle, since that indicates left/right
                angle = np.arctan(np.array([-past_vector[1], past_vector[0]]).dot(next_vector)/0.0001)
                
                if angle>0:
                    self.directions.append("right")
                else:
                    self.directions.append("left")
        
        return self.directions

if __name__ == "__main__":
    maze_obj = labyrinth.maze(19)
    maze_map = maze_obj.generate()
    obj = A_star(maze_map, liveplot=False)
    obj.start_algorithm()
    print(obj.return_direction_list())
    obj.plot_map()
    