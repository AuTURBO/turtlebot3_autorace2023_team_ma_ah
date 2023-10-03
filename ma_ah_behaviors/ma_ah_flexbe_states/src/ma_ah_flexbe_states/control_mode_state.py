#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

from enum import Enum
from std_msgs.msg import UInt8, Float64
from turtlebot3_autorace_msgs.msg import MovingParam
from std_msgs.msg import String

import math
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ControlModeState(EventState):
    '''
    Implements a FlexBE state that controls the lane of a robot.
    '''

    def __init__(self):
        super(ControlModeState, self).__init__(outcomes=['proceed', 'done'])

        # subscribes state
        self.sub_moving_state = ProxySubscriberCached('/control/moving/state', Float64)
        self.sub_odom = ProxySubscriberCached('/odom', Odometry)
        
        # publishers state
        self.pub_cmd_vel = ProxyPublisher('/cmd_vel', Twist)
        self.pub_max_vel = ProxyPublisher('/control/max_vel', Float64)
        self.pub_moving_complete = ProxyPublisher('/control/moving/complete', UInt8)



        #moving type enum
        self.TypeOfMoving = Enum('TypeOfMoving', 'idle left right forward backward')
        self.TypeOfState = Enum('TypeOfState', 'idle start stop finish')
        
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        
        #moving params
        self.moving_type = 3 # left : 1, right = 2, forward = 3, backward = 4
        self.moving_angluar = 0.0
        self.moving_linear = 1.0
        Logger.loginfo("get : moving_angluar : %f",self.moving_angluar)
        Logger.loginfo("get : moving_linear : %f", self.moving_linear)


        #moving flags
        self.is_step_start = False
        self.is_step_left = False
        self.is_step_right = False
        self.is_step_forward = False
        self.is_step_backward = False

        #moving internal valiables
        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.lastError = 0.0

        self.test = math.radians(90)
     
        msg_pub_max_vel = Float64()
        msg_pub_max_vel.data = 0.05
        self.pub_max_vel.publish(msg_pub_max_vel)

        loop_rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            if self.is_step_left == True:
                self.turn_left(self.moving_msg)
            elif self.is_step_right == True:
                self.turn_right(self.moving_msg)
            elif self.is_step_forward == True:
                self.go_forward(self.moving_msg)
            elif self.is_step_backward == True:
                self.go_backward(self.moving_msg)
            else:
                pass
            loop_rate.sleep()
        rospy.on_shutdown(self.fnShutDown)


    def on_enter(self, userdata):
        Logger.loginfo("Starting wheel odom test...")
        # if self._sub.has_msg("/traffic_sign"):
            
    def execute(self, userdata):
        pass
 
    def on_exit(self, userdata):
        # 이 메서드는 결과가 반환되고 다른 상태가 활성화될 때 호출됩니다.
        # on_enter에서 시작된 실행 중인 프로세스를 중지하는 데 사용할 수 있습니다.

        pass # 이 예시에서는 할 일이 없습니다.

    def on_start(self):
        # 이 메서드는 행동이 시작될 때 호출됩니다.
        # 가능하면, 일반적으로 사용된 리소스를 생성자에서 초기화하는 것이 더 좋습니다
        # 왜냐하면 무언가 실패하면 행동은 시작조차 되지 않을 것이기 때문입니다.

        # 이 예시에서는 이 이벤트를 사용하여 올바른 시작 시간을 설정합니다.
        pass

    def on_stop(self):

        
        # 이 메서드는 행동이 실행을 중지할 때마다 호출됩니다, 취소된 경우에도 마찬가지입니다.
        # 이 이벤트를 사용하여 요청된 리소스와 같은 것들을 정리하세요.

        pass # 이 예시에서는 할 일이 없습니다.
 



    def turn_left(self):
        Logger.loginfo("turn_left function is called")
        if self.is_step_start == False:
            self.lastError = 0.0
            self.desired_theta = self.current_theta + math.radians(self.moving_value_angular)
            self.is_step_start = True
        
        error = self.fnTurn()

        if math.fabs(error) < 0.05:
            Logger.loginfo("turn_left function is finished")
            self.step_completed()
    def turn_right(self, msg):
        Logger.loginfo("turn_right function is called")
        if self.is_step_start == False:
            self.lastError = 0.0
            self.desired_theta = self.current_theta - math.radians(self.moving_value_angular)
            self.is_step_start = True

        error = self.fnTurn()

        if math.fabs(error) < 0.05:
            Logger.loginfo("turn_right function is finished")
            self.step_completed()


    def go_forward(self):
        Logger.loginfo("go_forawrd function is called")
        if self.is_step_start == False:
            self.lastErorr = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y
            self.is_step_start = True

        error = self.fnStraight(self.moving_value_linear)

        if math.fabs(error) < 0.005:
            Logger.loginfo("go_forward function is finished")
            self.step_completed()


    def go_backward(self):
        Logger.loginfo("go_backward function is called")
        if self.is_step_start == False:
            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y
            self.is_step_start = True
        
        error = self.fnBackStraight(self.moving_value_linear)

        if math.fabs(error) < 0.005:
            Logger.loginfo("go_backward function is finished")
            self.step_completed()


    def fnTurn(self):
        err_theta = self.current_theta - self.desired_theta
        
        # Logger.loginfo("err_theta  desired_theta  current_theta : %f  %f  %f", err_theta, self.desired_theta, self.current_theta)
        Kp = 0.45
        Kd = 0.03

        angular_z = Kp * err_theta + Kd * (err_theta - self.lastError)
        self.lastError = err_theta

        twist = Twist()
        twist.linear.x = 0 #0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -(angular_z * 2)
        self.pub_cmd_vel.publish(twist)

        # Logger.loginfo("angular_z : %f", angular_z)

        return err_theta

    def fnStraight(self, desired_dist):   

        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
    
        # Logger.loginfo("error_pos2 = %f", err_pos)
        Kp = 0.04
        Kd = 0.05

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()

        if err_pos < 0:
            twist.linear.x = 0.1 #0.07
        else:
            twist.linear.x = -0.1

        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos


    def fnBackStraight(self, desired_dist):   

        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
    
        # Logger.loginfo("error_pos = %f", err_pos)
        Kp = 0.04
        Kd = 0.05

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()
      
        if err_pos < 0:
            twist.linear.x = -0.1 #0.07
        else:
            twist.linear.x = 0.1

        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)
            
    def cbOdom(self, odom_msg):
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)

        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y
    
    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta
    
    
    def fnShutDown(self):
        Logger.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)