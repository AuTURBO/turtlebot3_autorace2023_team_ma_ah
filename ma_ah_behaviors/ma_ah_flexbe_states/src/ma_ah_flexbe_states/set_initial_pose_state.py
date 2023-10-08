#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion

class SetInitialPoseState(EventState):
    """
    A state to set the initial pose of a robot in the map frame.

    -- x           float   Initial x-coordinate.
    -- y           float   Initial y-coordinate.
    -- orientation float   Initial orientation in radians.

    #> succeeded    bo
        pub_initial_pose = ProxyPublisher({"/initialpose": PoseWithCovarianceStamped})ol    True if the initial pose has been set successfully.
    """

    def __init__(self):
        """Constructor of the state."""
        super(SetInitialPoseState, self).__init__(outcomes=['succeeded'], input_keys=['initial_pose'])

        self.pub_initial_pose = ProxyPublisher({"/initialpose": PoseWithCovarianceStamped})

        self.succeeded = False

    def execute(self, userdata):
        """Execution of the state."""
        if self.succeeded:
            return 'succeeded'
    
    def on_enter(self, userdata):
        self.succeeded = False
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = "map"
        initial_pose_msg.header.stamp = rospy.Time.now()

        initial_pose_msg.pose.pose.position.x = userdata.initial_pose[0]
        initial_pose_msg.pose.pose.position.y = userdata.initial_pose[1]

        qt = transformations.quaternion_from_euler(0, 0, userdata.initial_pose[2])
        initial_pose_msg.pose.pose.orientation = Quaternion(*qt)
        self.pub_initial_pose.publish("/initialpose", initial_pose_msg)
        Logger.loginfo("Initial pose set to ({}, {}) with orientation ({})".format(userdata.initial_pose[0], userdata.initial_pose[1], userdata.initial_pose[2]))
        self.succeeded = True

    def on_exit(self, userdata):
        pass
    def on_start(self):
        pass
    def on_stop(self):
        pass
