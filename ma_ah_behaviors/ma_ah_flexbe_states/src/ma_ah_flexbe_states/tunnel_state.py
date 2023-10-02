#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from std_msgs.msg import String

class TunnelState(EventState):
    '''
    Example for a state to pass through a dark tunnel without lights and signs.
    This state listens to a topic for tunnel information and proceeds through the tunnel.

    <= proceed      Proceed through the dark tunnel.

    '''

    def __init__(self):
        # Declare outcomes by calling the super constructor with the corresponding arguments.
        super(TunnelState, self).__init__(outcomes=['proceed', 'done'])

        # Initialize class variables or state parameters here if needed.
        self._sub = ProxySubscriberCached({"/tunnel_info": String})
        self._tunnel_info = None

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Its main purpose is to check the condition of the state and trigger the corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._sub.has_msg("/tunnel_info"):
            tunnel_info = self._sub.get_last_msg("/tunnel_info").data
            Logger.loginfo("Tunnel info: {}".format(tunnel_info))

            # Assuming 'tunnel_info' is a string that indicates tunnel information.
            if tunnel_info == "dark":
                return 'proceed'
            else:
                Logger.loginfo("Finished tunnel Mode.")
                # Proceed by default if no information is available.
                return 'done'

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.

        Logger.loginfo('Entered state Tunnel')

    # You can define other state lifecycle methods like on_exit, on_start, and on_stop if needed.

