#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from std_srvs.srv import Empty  # 서비스 메시지 임포트
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller  # 서비스 클라이언트 임포트

class ClearCostmapsState(EventState):
    """
    FlexBE state to clear costmaps using a ROS service call.

    -- service_name    string    Name of the service to call.

    <= done     Service call was successful.
    <= failed   Failed to call the service.
    """

    def __init__(self):
        """Constructor."""
        super(ClearCostmapsState, self).__init__(outcomes=['done', 'failed'])
        self._service_name = '/move_base/clear_costmaps'
        self._service_client = None

    def execute(self, userdata):
        """Execute this state."""
        if self._service_client is not None:
            try:
                # Create a request object
                request = Empty._request_class()

                # Call the service
                response = self._service_client(request)
                Logger.loginfo("Service call successful.")
                return 'done'
            except Exception as e:
                Logger.logerr("Failed to call service: %s" % str(e))
                return 'failed'

    def on_enter(self, userdata):
        """Create a service client when entering the state."""
        try:
            # Create a service client
            self._service_client = rospy.ServiceProxy(self._service_name, Empty)
            rospy.loginfo("Waiting for service '%s'..." % self._service_name)
            rospy.wait_for_service(self._service_name)  # 서비스가 등록될 때까지 대기
            rospy.loginfo("Service '%s' is available." % self._service_name)
        except Exception as e:
            Logger.logerr("Failed to create service client: %s" % str(e))
            self._service_client = None

    def on_exit(self, userdata):
        """Cleanup when exiting the state."""
        self._service_client = None

    def on_start(self):
        pass

    def on_stop(self):
        pass