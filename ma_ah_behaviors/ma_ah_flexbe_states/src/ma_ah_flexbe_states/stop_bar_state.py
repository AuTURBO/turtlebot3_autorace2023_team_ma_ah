#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

from std_msgs.msg import String

class StopBarState(EventState):
    '''
    Example for a state to detect a stop bar and react accordingly.
    This state listens to a topic for stop bar information and stops the robot when the bar is down.

    <= stop             Stop bar is down.
    <= proceed          Stop bar is up.

    '''

    def __init__(self):
        # Declare outcomes by calling the super constructor with the corresponding arguments.
        super(StopBarState, self).__init__(outcomes=['stop', 'proceed'])

        # Initialize class variables or state parameters here if needed.
        self._sub = ProxySubscriberCached({"/stop_bar_status": String})
        self._stop_bar_status = None

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Its main purpose is to check the condition of the state and trigger the corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._sub.has_msg("/stop_bar_status"):
            stop_bar_info = self._sub.get_last_msg("/stop_bar_status").data
            Logger.loginfo("Stop bar status: {}".format(stop_bar_info))

            # Assuming 'stop_bar_info' is a string that indicates stop bar status.
            if stop_bar_info == "down":
                self._stop_bar_status = "down"
                return 'stop'
            elif stop_bar_info == "up":
                self._stop_bar_status = "up"
                return 'proceed'
        else:
            Logger.loginfo("No stop bar status available.")
            self._stop_bar_status = None
            return 'proceed'  # Proceed by default if no information is available.

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.

        Logger.loginfo('Entered state StopBar')

    # You can define other state lifecycle methods like on_exit, on_start, and on_stop if needed.

