#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

class macro_drive:
    def __init__(self):
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.move_cmd = Twist()

    def stop(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.move_cmd)
        rospy.sleep(1.0)

    def move(self, linear_x, angular_z, debug_msg, sleep_time):
        self.move_cmd.linear.x = linear_x
        self.move_cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(self.move_cmd)
        print(debug_msg)
        rospy.sleep(sleep_time)

    def move_robot(self):
        # ROS 노드 초기화
        rospy.init_node('robot_mover', anonymous=True)

        rospy.sleep(1)

        # 전진
        self.move(0.1, 0.0, "first forward", 1.7)
        self.stop()

        self.move(0.0, 0.5, "left", 1.4)
        self.stop()
        self.move(0.0, 0.5, "left", 1.4)
        self.stop()

        self.move(0.1, 0.0, "forward", 1.5)
        self.stop()
        self.move(0.1, 0.0, "forward", 1.5)
        self.stop()

        self.move(0.0, -0.5, "right", 1.2)
        self.stop()
        self.move(0.0, -0.5, "right", 1.2)
        self.stop()

        self.move(0.1, 0.0, "forward",1.2)
        self.stop()
        self.move(0.1, 0.0, "forward",1.2)
        self.stop()
        self.move(0.1, 0.0, "forward", 1.2)
        self.stop()
        
        self.move(0.0, -0.5, "right", 1.4)
        self.stop()
        self.move(0.0, -0.5, "right", 1.4)
        self.stop()

        self.move(0.1, 0.0, "forward", 1.4)
        self.stop()
        self.move(0.1, 0.0, "forward", 1.5)
        self.stop()



        self.move(0.0, 0.5, "left", 1.4)
        self.stop()
        self.move(0.0, 0.5, "left", 1.4)
        self.stop()

        self.stop()

if __name__ == '__main__':
    try:
        drive = macro_drive()
        drive.move_robot()
    except rospy.ROSInterruptException:
        pass
