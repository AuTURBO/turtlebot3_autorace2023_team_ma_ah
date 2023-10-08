#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

import math

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
            'back' : self.back
        }
        self._success = False
        self._fail = False
    def odom_update(self):
        # odometry = 
        odom_msg = self.sub_odom.get_last_msg("/odom")
        # 현재 robot의 yaw 값을 current_theta에 저장
        print(odom_msg)
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

        # 현재 robot의 위치를 current_pos에 저장
        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y
        self.current_pos = math.sqrt(self.current_pos_x**2 + self.current_pos_y**2)

    def drive(self, linear_velocity, angular_velocity):
        self.steer_angle.linear.x = linear_velocity
        self.steer_angle.angular.z = angular_velocity
        self.pub_cmd_vel.publish("/cmd_vel", self.steer_angle)
    

    def left_turn(self):
        Kp = 1.5
        Ki = 0.0
        Kd = 0.0
        self.lastError = 0
        current_distance = 0

        current_distance = self.current_theta 
        error = current_distance - 1.57
        control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error

        print("left_turn", self.current_theta)

        if abs(error) < 0.05:
            self.drive(0.0, -control)
            return True
            
    
    def right_turn(self):
        Kp = 1.5
        Ki = 0.0
        Kd = 0.0
        self.lastError = 0
        current_distance = 0

        print("right_turn")
        while 1:
            current_distance = self.current_theta 
            error = current_distance + 1.57
            control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
            self.lastError = error
            if abs(error) < 0.05:
                break
            self.drive(0.0, -control)
        return True
    def stop(self):
        print("stop")
        self.drive(0.0, 0.0)
        return True
    
    def go_straight(self):
        Kp = 1.5
        Ki = 0.0
        Kd = 0.0
        self.lastError = 0
        current_distance = 0

        print("go_straight")
        while 1:
            current_distance = self.current_pos 
            error = current_distance - 0.2
            control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
            self.lastError = error
            if abs(error) < 0.05:
                break
            self.drive(-control, 0.0)
        return True
    
    def back(self):
        Kp = 1.5
        Ki = 0.0
        Kd = 0.0
        self.lastError = 0
        current_distance = 0

        print("back")
        while 1:
            current_distance = self.current_pos 
            error = current_distance + 0.2
            control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
            self.lastError = error
            if abs(error) < 0.05:
                break
            self.drive(-control, 0.0)
        return True
    
    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Its main purpose is to check the condition of the state and trigger the corresponding outcome.
        # If no outcome is returned, the state will stay active.
        if self.control_dict[self.cmd_moving]() :
            return 'done'
        else :
            return 'procced'
               
 
        

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.
        # self._success = False
        # self._fail = False

        # self.cmd_moving = userdata.moving_info
        # # 3항 연산자
        # if self.control_dict[self.cmd_moving]() : 
        #     print("succesadasdasdasdss")
        #     self._success = True
        # else :
        #     print("faiasdasdasdasdasl")
        #     self._fail = True

        # Logger.loginfo('Entered state moving')
        pass

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