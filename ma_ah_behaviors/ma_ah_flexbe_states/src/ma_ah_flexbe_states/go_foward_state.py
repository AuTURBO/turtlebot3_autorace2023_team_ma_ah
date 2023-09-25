#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxyPublisher, ProxySubscriberCached
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class GoFowardState(EventState):

    def __init__(self, speed, travel_dist, obstacle_dist):
        super(GoFowardState, self).__init__(outcomes=['done', 'failed'])

        self._start_time = None
        self._speed = speed
        self._travel_dist = travel_dist
        self._obstacle_dist = obstacle_dist

        self.vel_topic = '/cmd_vel'
        self.scan_topic = '/scan'

        self.pub = ProxyPublisher({self.vel_topic: Twist})
        self.scan_sub = ProxySubscriberCached({self.scan_topic: LaserScan})
        self.scan_sub.set_callback(self.scan_topic, self.scan_callback)
        self.data = None  

    def execute(self, userdata):
        if not self.cmd_pub:
            return 'failed'
        
        Logger.loginfo('FWD obstacle distance is : {}'.format(min(self.data.ranges)))

        # elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
        # distance_travelled = (elapsed_time / 60.0) * self._speed
        
        # if distance_travelled >= self._travel_dist:
        #     return 'done'

        self.pub.publish(self.vel_topic, self.cmd_pub)
    
    def on_enter(self, userdata):
        Logger.loginfo("Drive FWD STARTED")

        self.cmd_pub = Twist()
        self.cmd_pub.linear.x = self._speed
        self.cmd_pub.angular.z = 0.0
        self._start_time = rospy.Time.now()

    def on_exit(self, userdata):
        self.cmd_pub.linear.x = 0.0
        self.pub.publish(self.vel_topic, self.cmd_pub)
        Logger.loginfo("Drive FWD ENDED")

    def on_start(self):
        Logger.loginfo("Drive FWD READY!")
        self._start_time = rospy.Time.now()

    def on_stop(self):
        Logger.loginfo("Drive FWD STOPPED!")

    def scan_callback(self, data):
        self.data = data

