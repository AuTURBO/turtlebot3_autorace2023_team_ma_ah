#!/usr/bin/env python

from flexbe_core import EventState, Logger
from subprocess import Popen, PIPE

class StartTurtlebot3NavigationState(EventState):
    """
    A state to start the turtlebot3_navigation package using roslaunch.

    -- package    string      Name of the ROS package to launch.
    -- launch_file    string  Name of the launch file to execute.

    <= done         Navigation is successfully started.
    <= failed       Failed to start navigation.

    """

    def __init__(self):
        """Constructor."""
        super(StartTurtlebot3NavigationState, self).__init__(outcomes=['done', 'failed'])
        self.package = "turtlebot3_navigation"
        self.launch_file = "turtlebot3_navigation.launch"
        self.process = None

    def execute(self, userdata):
        """Execute this state."""
        if self.process is not None:
            return 'done'
        else:
            Logger.loginfo("Starting Turtlebot3 navigation...")
            command = ['roslaunch', self.package, self.launch_file]
            try:
                self.process = Popen(command, stdout=PIPE, stderr=PIPE)
                return None
            except Exception as e:
                Logger.logerr("Failed to start Turtlebot3 navigation: {}".format(e))
                return 'failed'

    def on_enter(self, userdata):
        """Nothing special to do when entering the state."""
        pass

    def on_exit(self, userdata):
        pass
