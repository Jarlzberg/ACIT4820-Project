#! /usr/bin/env python

import rospy
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import LaserScan
from a_star_script import A_star
import time
import numpy as np
from cmd_vel_publisher import move_publisher

class turtle_bot:
    def __init__(self):
        self.mover = move_publisher()

        # load map
        path = __file__
        pkg_path = path.split("/src/main.py")[0]
        map_path = pkg_path+"/maps/map.npy"
        self.map = np.load(map_path)

        # get escape-route from A*-algorithm
        a_star_obj = A_star(self.map)
        a_star_obj.start_algorithm()
        self.directions = a_star_obj.return_direction_list()

        self.laser_data = LaserScan()
        
    def escape(self):
        # Drive forward until meeting a wall
        self.mover.move_robot("forward")
        rospy.sleep(0.05)
        wall = self.wall_detecter()

        while wall == False:
            self.mover.move_robot("forward")
            wall = self.wall_detecter()
            rospy.sleep(0.05)

        # robot has met a wall

        self.align_with_wall_front()    #correct for error

        # iterating through list of navigation-instructions
        for direction in self.directions:
            self.mover.turn_90(direction)   # turning either left or right
            self.align_with_wall_side(direction)    # aligning with a side-wall

            wall = self.wall_detecter()

            i = 0
            while wall == False:    # driving forward until reach a new wall
                self.mover.move_robot("forward")
                wall = self.wall_detecter()
                rospy.sleep(0.05)
                i+=1

                # if i >= 160:    # Stop to align with wall for long stretches
                #     self.align_with_wall_side(direction)
                #     i = 0

            # print("Wall!")

            self.align_with_wall_front()


        print("Out of loop")

        # done with instructions
        # drive forward until out of maze. 
        while self.check_if_escaped() == 0:
            self.mover.move_robot("forward")
            rospy.sleep(0.5)

        print("Turtlebot successfully escaped the maze!")
        self.mover.move_robot("stop")

        # Escape-program is done. 

    def check_if_escaped(self):
        # Robot has escaped when it cannot see walls in front, or to the sides. 
        self.laser = rospy.wait_for_message("/scan", LaserScan)
        front = self.laser[0]
        right = self.laser[90]
        left = self.laser[270]

        if front == float("Inf") and left == float("Inf") and right == float("Inf"):
            return 1
        else:
            return 0

    def align_with_wall_front(self):
        """ A simple p-controller that will align robot with a front-facing wall """
        err = self.get_error_front()

        while np.abs(err)>1:
            self.correct_error(err)
            err = self.get_error_front()


    def align_with_wall_side(self, direction):
        """ A simple p-controller that will align the robot with a parallel wall """

        err = self.get_error_side(direction)

        i = 0 
        while np.abs(err)>1:
            if i < 10:
                self.correct_error(err)
                err = self.get_error_side(direction)

            elif i < 20:
                new_direction = "left"
                if direction == "left":
                    new_direction = "right"

                self.correct_error(err)
                err = self.get_error_side(new_direction)

            else:
                break

            i+=1

    def get_error_front(self):
        """ Will return the orientation-offset between the robot and the wall infront of it in degrees. """

        self.mover.move_robot("stop")
        rospy.sleep(0.05)

        self.laser = rospy.wait_for_message("/scan", LaserScan)


        if self.laser.ranges[-10] != float("Inf") and self.laser.ranges[10] != float("Inf"):
            # converting from polar coordinates to cartesian
            x1 = self.laser.ranges[-10]*np.cos(-10*np.pi/180)
            y1 = self.laser.ranges[-10]*np.sin(-10*np.pi/180)
            x2 = self.laser.ranges[10]*np.cos(10*np.pi/180)
            y2 = self.laser.ranges[10]*np.sin(10*np.pi/180)

            delta_x = x2 - x1
            delta_y = y2 - y1

            if abs(delta_x) >= 0.0001:
                degrees = np.arctan(delta_y/delta_x)*180/np.pi - 90
            else:
                degrees = np.arctan(delta_y/0.0001)*180/np.pi - 90
            
            if delta_x <= 0 and delta_y <= 0:
                degrees = degrees
            elif delta_x <= 0 and delta_y >= 0:
                degrees = 180 + degrees
        
        else:
            degrees = 0
        # print("delta_x: ",delta_x)
        # print("delta_y: ",delta_y)

        # print(degrees)
        return degrees

    def get_error_side(self, direction):
        """ Will return the orientation-offset between the robot and either the left- or right wall. 
            Which wall to use is decides by the direction argument.
        """

        a1 = 0
        a2 = 0

        self.mover.move_robot("stop")
        rospy.sleep(0.05)

        # Deciding which sensor-data to use
        if direction == "right":
            a1 = 70
            a2 = 110

        elif direction == "left":
            a2 = 250
            a1 = 290

        self.laser = rospy.wait_for_message("/scan", LaserScan)

        if self.laser.ranges[a1] != float("Inf") and self.laser.ranges[a2] != float("Inf"):
        # converting from polar to cartesian coordinates
            x1 = self.laser.ranges[a1]*np.cos(a1*np.pi/180)
            y1 = self.laser.ranges[a1]*np.sin(a1*np.pi/180)
            x2 = self.laser.ranges[a2]*np.cos(a2*np.pi/180)
            y2 = self.laser.ranges[a2]*np.sin(a2*np.pi/180)

            delta_x = x2 - x1
            delta_y = y2 - y1

            if abs(delta_x) >= 0.0001:
                degrees = np.arctan(delta_y/delta_x)*180/np.pi - 180
            else:
                degrees = np.arctan(delta_y/0.0001)*180/np.pi - 180

            # since arctan() cant distinguish {+/+, -/-} and {+/-, -/+}, it is necessary to manually add 180* for {-/+} and {-/-}
            if delta_x >= 0 and delta_y >= 0:
                degrees = degrees
            elif delta_x <= 0 and delta_y >= 0:
                degrees = 180 + degrees
            elif delta_x >= 0 and delta_y <= 0:
                degrees = degrees
            elif delta_x <= 0 and delta_y <=0:
                degrees = 180 + degrees

        else:
            degrees = 0

        return degrees


    def correct_error(self, degrees):
        """ Rotating robot to compensate for error """
        rot_time = (2.2/90) * np.abs(degrees)

        if degrees > 0:
            self.mover.move_robot("left")
        else:
            self.mover.move_robot("right")

        rospy.sleep(rot_time)
        self.mover.move_robot("stop")
        rospy.sleep(0.1)

    def wall_detecter(self):
        """ Will return True if close to a wall """
        rospy.wait_for_service('/laser_scan_service')
        crash = rospy.ServiceProxy('/laser_scan_service', SetBool)
        result = crash()
        
        return result.success
        
if __name__ == '__main__':
    rospy.init_node('start_main_node')
    turtle_bot_object = turtle_bot()
    turtle_bot_object.escape()
    rospy.spin()