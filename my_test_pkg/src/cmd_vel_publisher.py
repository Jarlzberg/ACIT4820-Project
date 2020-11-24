#! /usr/bin/env python
import rospy                                          
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class move_publisher(object):
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.speed = 0.25
        self.rot_90 = 0.51
        self.rot_time = 3.10
        self.state = ""

    def move_robot(self, direction):
        if direction == "forward":
            self.twist.linear.x = self.speed
            self.twist.angular.z = 0
            self.state = direction
            
        elif direction == "backward":
            self.twist.linear.x = -self.speed
            self.twist.angular.z = 0
            print("backward")
            self.state = direction
        
        elif direction == "left":
            self.twist.linear.x = 0
            self.twist.angular.z = self.rot_90
            self.state = direction
        
        elif direction == "right":
            self.twist.linear.x = 0
            self.twist.angular.z = -self.rot_90
            self.state = direction
        
        elif direction == "stop":
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.state = direction


        while self.pub.get_num_connections() < 1:
            rospy.loginfo_throttle(2, "Waiting for subscribers on /ardrone/land ..")
            rospy.sleep(0.1)

        self.pub.publish(self.twist)
        
    def set_state(self,state):
        self.state = state
    
    def get_state(self):
        return state

    def turn_90(self, direction):
        self.move_robot("stop")
        rospy.sleep(0.5)

        if direction == "left":
            self.move_robot("left")

        elif direction == "right":
            self.move_robot("right")
        
        rospy.sleep(self.rot_time)

        self.move_robot("stop")
        rospy.sleep(0.1)

    
if __name__ == '__main__':
    rospy.init_node('cmdVelPubNode')
    turtle = move_publisher()
    
    turtle.turn_90("left")
    turtle.turn_90("left")

    #turtle.move_robot("forward")
    # rospy.sleep(5)
    # turtle.turn_90("right")
    # turtle.move_robot("forward")
    # rospy.sleep(3)
    # turtle.turn_90("left")
    # turtle.move_robot("forward")

    rospy.sleep(1.5)
    turtle.move_robot("stop")
    rospy.spin()
