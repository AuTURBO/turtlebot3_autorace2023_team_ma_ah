#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from tf import transformations

"""
Created on 11/19/2015

@author: Spyros Maniatopoulos
"""
# 터미널 명령어
# rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: 'map'
# pose:
#   position:
#     x: 0.8
#     y: 3.65
#     z: 0.0
#   orientation:
#     x: 0.0
#     y: 0.0
#     z: 3.14
#     w: 0.0" 


#MoveBaseState 클래스 정의
class MoveBaseState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.

    ># waypoint     Pose2D      Target waypoint for navigation.

    <= arrived                  Navigation to target pose succeeded.
    <= failed                   Navigation to target pose failed.
    """

    def __init__(self):
        """Constructor"""
				
		#상태의 결과와(outcome)와 입력 키 (input_keys) 설정
		#이 상태는 'arrived'와 'failed' 두 가지 결과를 반환하고, 'waypoint' 입력 키를 사용
        super(MoveBaseState, self).__init__(outcomes = ['arrived', 'failed'],
                                            input_keys = ['waypoint'])
				
		#action_topic 초기화
        self._action_topic = "/move_base"
				
		#Action 클라이언트와 관련된 변수를 초기화
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})

        self._arrived = False
        self._failed = False

	#execute 메서드는 이벤트 상태가 실행될 때 호출되며, 움직임 명령의 결과를 대기하고 이에 따른 결과를 반환합니다.
    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

		#먼저, _arrived와 _failed 변수를 확인하여 현재 상태가 이미 목적지에 도착했거나 실패한 상태인지를 확인
        if self._arrived:
            return 'arrived'
        if self._failed:
            return 'failed'

		#_client 객체의 has_result 메서드를 호출하여 움직임 명령의 결과가 있는지 확인합니다. 움직임 명령의 결과가 있을 경우 아래의 코드 블록을 실행
        if self._client.has_result(self._action_topic):

			#움직임 명령의 상태를 get_state 메서드를 사용하여 가져옵니다. 
			#이 상태는 GoalStatus 메시지에 정의되어 있으며, GoalStatus.SUCCEEDED는 목적지에 성공적으로 도착했음을 나타냄
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._arrived = True
                return 'arrived'
			    #움직임 명령의 상태가 GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.RECALLED, 또는 GoalStatus.ABORTED 중 하나에 해당하는 경우,
				#아래의 코드 블록을 실행
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]: 
				#Logger.logwarn 메서드를 사용하여 로그에 경고 메시지를 출력합니다. 이 메시지는 실패한 이유를 나타냄
                Logger.logwarn('Navigation failed: %s' % str(status))
                self._failed = True
                return 'failed'

	#MoveBaseState 상태가 진입할 때 호출되며 로봇의 이동 명령을 생성하고 전송하는 역할
    def on_enter(self, userdata):
        """Create and send action goal"""
				
				#변수 초기화 & 플래그 설정
        self._arrived = False
        self._failed = False

        # Create and populate action goal
		# 번역 : MoveBaseGoal 객체를 생성 이 객체는 이동 명령의 목표
        goal = MoveBaseGoal()
			
		#userdata.waypoint에서 가져온 좌표를 사용하여 Point 객체 pt를 생성.
		# 이 pt는 목표 지점의 x와 y 좌표를 나타냄
        print(userdata.waypoint)
        pt = Point(x = userdata.waypoint[0], y = userdata.waypoint[1])

		#userdata.waypoint에서 가져온 각도 값을 사용하여 Quaternion 객체 qt를 생성 
		#이 qt는 목표 지점의 방향을 나타냅니다.
        # qt = transformations.quaternion_from_euler(0, 0, userdata.waypoint[2])

        qt = [0, 0, userdata.waypoint[2], userdata.waypoint[3]]
				
		#goal 객체의 target_pose 속성을 설정하여 목표 위치와 방향을 지정
        goal.target_pose.pose = Pose(position = pt,
                                     orientation = Quaternion(*qt))
				
		#goal 객체의 header.frame_id 속성을 "odom"으로 설정
        goal.target_pose.header.frame_id = "map"
        # goal.target_pose.header.stamp.secs = 5.0

        # Send the action goal for execution
		#위에서 설정한 목표를 이용하여 self._client.send_goal 메서드를 사용하여 이동 명령을 전송
        try:
            self._client.send_goal(self._action_topic, goal)
        except Exception as e:
		#이동 명령 전송이 실패하면 Logger.logwarn를 사용하여 경고 메시지를 출력
            Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
            self._failed = True


    #cancel_active_goals 메서드는 현재 활성화된 움직임 명령을 취소하는 역할        
    def cancel_active_goals(self):
		#이동 명령을 처리할 수 있는지 여부를 확인
        if self._client.is_available(self._action_topic):
			#현재 활성화된 움직임 명령이 있는지 확인
            if self._client.is_active(self._action_topic):
				#움직임 명령의 결과가 아직 없는지 확인, 움직임 명령이 이미 완료된 경우에는 취소할 필요 없음
                if not self._client.has_result(self._action_topic):
				#위 조건 만족시, 현재 활성화된 움직임 명령을 취소
                    self._client.cancel(self._action_topic)
				#로그에 취소 메시지를 출력
                    Logger.loginfo('Cancelled move_base active action goal.')

		#on_exit 메서드는 상태가 종료될 때 호출되는 메서드
    def on_exit(self, userdata):
				#상태가 종료될 때 활성화된 움직임 명령을 취소
        self.cancel_active_goals()

		#메서드는 상태가 중지될 때 호출되는 메서드
    def on_stop(self):
        self.cancel_active_goals() 