# ACIT4820

Repository for project in ACIT4820 - Applied Robotics and Autonomous Systems

Python-scripts are located in my_test_pkg/src

Use launch-file "start_simulation.launch" to start the world-generator and initiate gazebo. 
Use launch-file "start_main.launch" to start the escape-program. 

This project depends on the Turtlebot3-package from ROBOTIS. https://github.com/ROBOTIS-GIT/turtlebot3

The following lines might be necessary to add to the .bashrc-file, for the program to run. 

"

export GAZEBO_MODEL_PATH=/usr/share/gazebo-7/models:${GAZEBO_MODEL_PATH}'

export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-7/plugins:${GAZEBO_PLUGIN_PATH}

export TURTLEBOT3_MODEL=burger

source /home/*insert username*/catkin_ws/devel/setup.bash

"
