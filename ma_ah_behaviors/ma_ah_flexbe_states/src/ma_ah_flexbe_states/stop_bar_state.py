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
        super(StopBarState, self).__init__(outcomes=['proceed', 'done'])

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
                return 'proceed'
            elif stop_bar_info == "up":
                Logger.loginfo("Finished Stop bar mode.")
                self._stop_bar_status = "up"
                return 'done'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.

        Logger.loginfo('Entered state StopBar')

    # You can define other state lifecycle methods like on_exit, on_start, and on_stop if needed.


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