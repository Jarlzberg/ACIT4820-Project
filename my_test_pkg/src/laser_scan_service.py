#! /usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist 
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import LaserScan
import numpy as np

class laser_server(object):
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.crash_service = rospy.Service('/laser_scan_service', SetBool, self.crash_callback)
        self.laser_data = LaserScan()
        self.distance = 0.63

        rospy.spin() # maintain the service open.

    def crash_callback(self, request):
        response = SetBoolResponse()
        avg_lsr = (self.laser_data.ranges[-2] + self.laser_data.ranges[2])/2

        if avg_lsr <= self.distance:
            # close enough to wall
            response.success = True
        else:
            response.success = False
        
        return response

    def laser_callback(self, msg):                                                                                    
        self.laser_data = msg

    def get_laser_data(self):
        return self.laser_data.ranges

if __name__ == "__main__":
    rospy.init_node('start_laser_server_node')
    laser_object = laser_server()
    rospy.spin()