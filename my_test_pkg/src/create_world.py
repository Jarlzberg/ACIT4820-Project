#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import labyrinth
import sys
from string import Template
import numpy as np
import time

start_time = time.time()

# Accepted labyrinth dimensions
sizes = [20, 30, 40]

link_format = None
model_format = None

# Getting path to link- and model-description-files
self_path = __file__
dir_path = self_path.split("src/create_world.py")[0]
link_format_path = dir_path + "templates/link_format.txt"
model_format_path = dir_path + "templates/model_format.txt"


# Getting the text-template for the link-description
with open(link_format_path,"r") as f:
    link_format = f.read()
    f.close()

# Getting the text-template for the model-description
with open(model_format_path,"r") as f:
    model_format = f.read()
    f.close()


def add_one_block_link(x,y):
    """ Fills in the link-description for one maze-block """
    name = "Unit_box_"+str(x)+"_"+str(y) # generating standardized model-name
    template = Template(link_format)
    output = template.substitute({ # filling in $linkname, $collisionname, $visualname, $x and $y accordingly
        'linkname':name,
        'collisionname':name+"_collision",
        'visualname':name+"_visual",
        'x':str(x),
        'y':str(y)})

    return output

def add_one_block_model(x,y):
    """ Fills in the model-description for one maze-block """
    name = "Unit_box_"+str(x)+"_"+str(y) # generating standardized model-name
    template = Template(model_format)
    output = template.substitute({ # filling in $linkname, $x and $y accordingly
        'linkname':name,
        'x':str(x),
        'y':str(y)})

    return output



def create_world_file(maze):
    """ Create a new world-file (rewriting the maze_world.world each time) 
        Adding 1x1x1 blocks to fill in the maze by appending descriptions to the template
        world-file. 
    """

    path = __file__
    pkg_path = path.split("/src/create_world.py")[0]
    empty_world_path = pkg_path+"/worlds/30x30_world.world"
    new_path = pkg_path + "/worlds/maze_world.world"
    new_world = []    

    # reading the template world file, which only contains border-walls
    with open(empty_world_path,"r") as f:
        old_world = f.readlines()
        f.close()
    
    # Iterating throught the template world file, and appending object-descriptions at the right places. 
    for line in old_world:
        if "</spherical_coordinates>" in line:
            new_world.append(line) # append line first

            # Adding all link-descriptions
            for i in range(len(maze)):
                for j in range(len(maze)):
                    # Iterate through maze-array
                    if maze[i][j]: # fill in block if 1.
                        block_input = add_one_block_link(j-14, -i+14).splitlines()
                        #block_input = add_one_block_link(j-13.52, -i+14).splitlines()
                        for l in block_input:
                            l2 = l+"\n"
                            new_world.append(l2)
                
        elif "<light name='sun'>" in line:

            #Adding all model-descriptions
            for i in range(len(maze)):
                for j in range(len(maze)):
                    # Iterate through maze-array
                    if maze[i][j]: # fill in block if 1. 
                        block_input = add_one_block_model(j-14, -i+14).splitlines()
                        #block_input = add_one_block_model(j-13.52, -i+14).splitlines()
                        for l in block_input:
                            l2 = l+"\n"
                            new_world.append(l2)

            new_world.append(line) # append line after

        else:
            # Copy directly from template-file
            new_world.append(line)

    # Write the filled-in template to the new maze_world.world file. 
    with open(new_path,"w+") as f:
        for line in new_world:
            f.write(line)
        
        f.close()
    


### --- Ideally it would be possible specify the dimensions of the maze when launching,
#       but this feature is not yet completed --- ###

# rospy.init_node("maze_maker_node")
# dimension = rospy.myargv(argv=sys.argv)
# x_bias = 0
# y_bias = 0
# if int(dimension[1]) == 20:
#     x_bias = 9.4
#     y_bias = 9.2

# elif int(dimension[1]) == 30:
#     x_bias = 14
#     y_bias = 14

# elif int(dimension[1]) == 40:
#     x_bias = 19.4
#     y_bias = 19.2

# if int(dimension[1]) not in sizes:
#     print("Invalid dimension")
#     sys.exit(1)
# maze_obj = labyrinth.maze(int(dimension[1])-1)

use_imported_map = False

if use_imported_map:
    # Use a previously generated map. Must be .npy file
    imported_map = np.load(dir_path+"maps/obstacles_map4.npy")
    create_world_file(imported_map)

else:
    # Generate a new labyrinth, using the functions from labyrinth.py
    maze_obj = labyrinth.maze(30-1)
    maze = maze_obj.generate()
    maze_obj.save_labyrinth()
    create_world_file(maze)
    print("Completion time: ", time.time() - start_time)
    maze_obj.init_plot()
    maze_obj.plot_maze()    




### ---- OLD TECHNIQUE ---- ###
### ---- Abandoned due to slow spawning, and taxing on simulation time ---- ###

# rospy.wait_for_service("/gazebo/spawn_sdf_model")
# for i in range(len(maze)):
#     for j in range(len(maze[i])):
#         if maze[i][j]:
#             try:
#                 spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
#                 spawner('block'+str(i)+"x"+str(j), 
#                         open("/home/jarlzberg/catkin_ws/src/my_test_pkg/models/blockmodel/model.sdf",'r').read(),
#                         "/block"+str(i)+"x"+str(j),
#                         Pose(position= Point(i-x_bias, j-y_bias, 0),
#                         orientation=Quaternion(0,0,0,0)),
#                         "world")

#             except rospy.ServiceException as e:
#                 print("Service call failed: ",e)
