#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from flexbe_core.proxy import ProxySubscriberCached

from std_msgs.msg import String


class TrafficSign(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(TrafficSign, self).__init__(outcomes = ['intersection_sign', 'obstacle_detection', 'parking_detection', 'stop_bar_status', 'tunnel_info'])
		
		
		# Store state parameter for later use.

		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.

		self._sub = ProxySubscriberCached({"/filtered/detection": String})
		#self._sign = 'lane_control'
		self._sign = None
		
		
	def execute(self, userdata):
		# 이 메서드는 상태가 활성 상태인 동안 주기적으로 호출됩니다.
		# 주요 목적은 상태 조건을 확인하고 상응하는 결과를 트리거하는 것입니다.
		# 결과가 반환되지 않으면 상태는 활성 상태로 유지됩니다.
		if self._sub.has_msg("/filtered/detection"):
			self._sign = self._sub.get_last_msg("/filtered/detection").data
			

			print("self._sign : ", self._sign, "type(self._sign) : ",  type(self._sign))
			
			if self._sign == "['intersection']":
				print("intersection mode")
				return 'intersection_sign'
			elif self._sign == "['obstacle']":
				print("obstacle mode")
				return 'obstacle_detection'
			elif self._sign == "['parking']":
				print("parking mode")
				return 'parking_detection'
			elif self._sign == "['stop']":
				print("stop mode")
				return 'stop_bar_status'
			elif self._sign == "['tunnel']":
				print("tunnel mode")
				return 'tunnel_info'
			elif self._sign == "['boom_barrier']":
				print("boom_barrier mode")
				return 'boom_barrier_info'
			elif self._sign == "['left']":
				print("left mode")
				return 'left_info'
			elif self._sign == "['right']":
				print("right mode")
				return 'right_info'
			elif self._sign == "['no_entry']":
				print("no_entry mode")
				return 'no_entry_info'

		# else:
		# 	Logger.loginfo("no traffic sign")
		# 	return 'lane_control'


	def on_enter(self, userdata):
		# 이 메서드는 상태가 활성화될 때 호출됩니다, 즉, 이 상태로의 다른 상태에서의 전환을 취할 때입니다.
		# 주로 이 상태와 관련된 행동을 시작하는 데 사용됩니다.

		# 다음 코드는 행동 로거가 어떻게 작동하는지 보여주기 위한 것입니다.
		# 행동 로거에 의해 기록된 텍스트는 운영자에게 전송되고 GUI에 표시됩니다.
		Logger.loginfo('Entered state TrafficSign')





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
