#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

import math, tf, os, time

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class MovingControlState(EventState):
    '''
    Example for a state to detect parking spots.
    This state listens to a topic for parking information and reacts accordingly.

    <= parking_proceed     Parking spot available.
    <= no_parking_spot            No parking spot available.

    '''
    def __init__(self):
        # Declare outcomes by calling the super constructor with the corresponding arguments.
        super(MovingControlState, self).__init__(outcomes=['procced', 'done'], input_keys=['moving_info'])

        self.sub_odom = ProxySubscriberCached({"/odom": Odometry})
        self.sub_scan = ProxySubscriberCached({"/scan": LaserScan})
        self.pub_cmd_vel = ProxyPublisher({"/cmd_vel": Twist})

        self.steer_angle = Twist()
        self.current_pos = 0
        self.current_theta = 0
        self.lastError = 0
        self.last_current_theta = 0
        self.last_current_pos = 0
        self.cmd_moving = None

        self.target_distance = 0.2
        self.current_distance = 0.0

        self.target_theta = 1.57/2
        self.current_theta = 0.0

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.is_moving = False
        


        self.distance = 0.2
        # Initialize class variables or state parameters here if needed.
        self.control_dict = {
            # 일정거리 직진 
            'go' : self.go_straight,
            # 정지
            'stop' : self.stop,
            # 좌회전
            'left' : self.left_turn,
            # 우회전
            'right' : self.right_turn,
            # 후진
            'back' : self.back,

            'odom' : self.odom_update,
        }
        self._success = False
        self._fail = False
    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta
    def odom_update(self):
        # odometry = 
        odom_msg = self.sub_odom.get_last_msg("/odom")

        if not self.is_moving:
            self.start_x = odom_msg.pose.pose.position.x
            self.start_y = odom_msg.pose.pose.position.y
            self.start_theta = odom_msg.pose.pose.orientation.z
            self.is_moving = True
        
        delta_x = odom_msg.pose.pose.position.x - self.start_x
        delta_y = odom_msg.pose.pose.position.y - self.start_y
        self.current_distance = math.sqrt(delta_x**2 + delta_y**2)
        self.current_theta = odom_msg.pose.pose.orientation.z - self.start_theta

    def drive(self, linear_velocity, angular_velocity):
        self.steer_angle.linear.x = linear_velocity
        self.steer_angle.angular.z = angular_velocity
        self.pub_cmd_vel.publish("/cmd_vel", self.steer_angle)
    

    def left_turn(self):
        self.control_dict['odom']()
        Kp = 1.5
        Ki = 0.0
        Kd = 0.0
        self.lastError = 0


        error = self.current_theta - self.target_theta
        control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error

        # print("left_turn", self.current_theta)
        print("error ", error)
        if abs(error) < 0.05:
            self.drive(0.0, 0.0)
            self.is_moving = False
            return True
        else:
            self.drive(0.0, -control)
            return False
            
    
    def right_turn(self):
        self.control_dict['odom']()
        Kp = 1.5
        Ki = 0.0
        Kd = 0.0
        self.lastError = 0

        error = self.current_theta + self.target_theta
        control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error

        print("left_turn", self.current_theta)

        if abs(error) < 0.05:
            self.drive(0.0, 0.0)
            self.is_moving = False
            return True
        else:
            self.drive(0.0, -control)
            return False







    def stop(self):
        print("stop")
        self.drive(0.0, 0.0)
        return True
    
#

    def go_straight(self):
        self.control_dict['odom']()
        Kp = 1.5
        Ki = 0.0
        Kd = 0.0
        self.lastError = 0

        error = self.current_distance - self.target_distance
        control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error

        # print("diff", self.current_pos - self.last_current_pos)
        print("error ", error)

        if abs(error) < 0.05:
            self.drive(0.0, 0.0)
            self.is_moving = False
            return True
        else: 
            self.drive(-control, 0.0)
            return False
        


    def back(self):
        self.control_dict['odom']()
        Kp = 1.5
        Ki = 0.0
        Kd = 0.0
        self.lastError = 0

        error = self.current_distance - self.target_distance
        control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error

        print("back", self.current_pos)

        if abs(error) < 0.05:
            self.drive(0.0, 0.0)
            self.is_moving = False
            return True
        else:
            self.drive(control, 0.0)
            return False
        

    def execute(self, userdata):
  
        if self._success:
            return 'done'
        elif self._fail:
            return 'procced'
 
        

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.
        self._success = False
        self._fail = False
        
        self.control_dict['odom']()
        self.cmd_moving = userdata.moving_info
        # # # 3항 연산자
        if self.control_dict[self.cmd_moving]() : 
            print("succesadasdasdasdss")
            self._success = True
        else :
            print("faiasdasdasdasdasl")
            self._fail = True

        # Logger.loginfo('Entered state moving')
        # pass

    # You can define other state lifecycle methods like on_exit, on_start, and on_stop if needed.

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