#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String

class TrafficLightState(EventState):
    '''
    Example for a state to detect traffic light signals and respond accordingly.
    This state listens to a topic for traffic light information and reacts based on the signal.

    <= green_light       Green traffic light.
    <= red_or_yellow_light  Red or yellow traffic light.
    <= no_signal         No traffic light signal.

    '''

    def __init__(self):
        # Declare outcomes by calling the super constructor with the corresponding arguments.
        super(TrafficLightState, self).__init__(outcomes=['green_light', 'red_or_yellow_light', 'no_signal'])

        # Initialize class variables or state parameters here if needed.
        self._sub = ProxySubscriberCached({"/traffic_light": String})
        self._traffic_light_signal = None





    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Its main purpose is to check the condition of the state and trigger the corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._sub.has_msg("/traffic_light"):
            traffic_light_info = self._sub.get_last_msg("/traffic_light").data
            Logger.loginfo("Traffic light signal: {}".format(traffic_light_info))

            # Assuming 'traffic_light_info' is a string that indicates traffic light signal.
            if traffic_light_info == "green":
                self._traffic_light_signal = "green"
                return 'green_light'
            elif traffic_light_info in ["red", "yellow"]:
                self._traffic_light_signal = "red_or_yellow"
                return 'red_or_yellow_light'
        else:
            Logger.loginfo("No traffic light signal available.")
            self._traffic_light_signal = None
            return 'no_signal'

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.

        Logger.loginfo('Entered state TrafficLight')

    # You can define other state lifecycle methods like on_exit, on_start, and on_stop if needed.

